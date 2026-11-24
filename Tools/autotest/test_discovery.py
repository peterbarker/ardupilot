'''
Discovery of autotest tests which live in their own directories.

Each test suite class names a directory of tests (its "test root", e.g.
ArduCopter_Tests).  Every subdirectory of that root holds one group of
tests plus the resources (missions, fences, scripts...) those tests use:

  ArduPlane_Tests/Deadreckoning/Deadreckoning.py
  ArduPlane_Tests/Deadreckoning/flaps.txt

The Python file defines a class with the same name as the directory and
file, deriving from TestGroup.  Methods decorated with @testcase are tests;
other methods are helpers used by those tests.  Every test name must start
with the group name, so a group holds a single test or a family of tests
sharing code (Deadreckoning, DeadreckoningNoAirSpeed).

The methods of every group are installed onto the suite class, so they run
with the suite instance as "self" exactly as methods written in the
vehicle's test file do.

AP_FLAKE8_CLEAN
'''

import ast
import importlib.util
import inspect
import os
import sys

testdir = os.path.dirname(os.path.realpath(__file__))


class TestDiscoveryError(Exception):
    '''raised when a test group breaks the layout rules'''
    pass


class TestGroup(object):
    '''base class for a group of tests living in their own directory'''
    pass


class TestCaseInfo(object):
    '''information attached to a test function by @testcase'''
    def __init__(self, buckets, attempts=1, speedup=None, kwargs=None, disabled=None):
        self.buckets = buckets
        self.attempts = attempts
        self.speedup = speedup
        self.kwargs = kwargs
        self.disabled = disabled


def testcase(bucket, attempts=1, speedup=None, kwargs=None, disabled=None):
    '''decorator marking a TestGroup method as a test

    bucket is the CI job (a tester_class_map entry in autotest.py, without
    its "test." prefix) the test runs in, or a list of them.  disabled is
    a reason string; disabled tests are listed but skipped.'''
    if isinstance(bucket, str):
        buckets = (bucket,)
    else:
        buckets = tuple(bucket)
    if len(buckets) == 0:
        raise TestDiscoveryError("testcase needs at least one bucket")

    def decorator(function):
        function._autotest_testcase = TestCaseInfo(
            buckets,
            attempts=attempts,
            speedup=speedup,
            kwargs=kwargs,
            disabled=disabled,
        )
        return function
    return decorator


class DiscoveredTest(object):
    '''a test found in a test root'''
    def __init__(self, name, function, info, group_dir):
        self.name = name
        self.function = function
        self.info = info
        self.group_dir = group_dir  # relative to testdir, trailing slash


class TestRoot(object):
    '''every test group found in one test root directory'''
    def __init__(self, root):
        self.root = root
        self.tests = {}     # test name -> DiscoveredTest
        self.members = {}   # attribute name -> (value, group name)

    def buckets(self):
        ret = set()
        for t in self.tests.values():
            ret.update(t.info.buckets)
        return ret


_roots = {}


def _zero_argument_super_users(filepath, group_name):
    '''return names of methods of class group_name in filepath which call
    super() without arguments (CPython gives every method mentioning
    super a __class__ cell, so the code object cannot tell us)'''
    with open(filepath) as f:
        tree = ast.parse(f.read(), filepath)

    def calls_super(node):
        for child in ast.iter_child_nodes(node):
            if isinstance(child, ast.ClassDef):
                # super() in a class nested in a method is that class's
                continue
            if (isinstance(child, ast.Call) and isinstance(child.func, ast.Name) and
                    child.func.id == "super" and len(child.args) == 0):
                return True
            if calls_super(child):
                return True
        return False

    ret = set()
    for node in tree.body:
        if not isinstance(node, ast.ClassDef) or node.name != group_name:
            continue
        for item in node.body:
            if isinstance(item, (ast.FunctionDef, ast.AsyncFunctionDef)) and calls_super(item):
                ret.add(item.name)
    return ret


def _load_group(root, group_name, filepath):
    module_name = "autotest_tests.%s.%s" % (root, group_name)
    spec = importlib.util.spec_from_file_location(module_name, filepath)
    module = importlib.util.module_from_spec(spec)
    sys.modules[module_name] = module
    spec.loader.exec_module(module)
    group = getattr(module, group_name, None)
    if group is None or not inspect.isclass(group) or not issubclass(group, TestGroup):
        raise TestDiscoveryError(
            "%s must define class %s(TestGroup)" % (filepath, group_name))
    return group


def _collect(root):
    ret = TestRoot(root)
    rootpath = os.path.join(testdir, root)
    if not os.path.isdir(rootpath):
        return ret
    for group_name in sorted(os.listdir(rootpath)):
        group_path = os.path.join(rootpath, group_name)
        filepath = os.path.join(group_path, group_name + ".py")
        if not os.path.isfile(filepath):
            # a resource directory without tests of its own
            continue
        group = _load_group(root, group_name, filepath)
        super_users = _zero_argument_super_users(filepath, group_name)
        group_dir = root + "/" + group_name + "/"
        found_test = False
        for (name, value) in vars(group).items():
            if name.startswith("__") and name.endswith("__"):
                continue
            where = "%s.%s" % (filepath, name)
            if name in ret.members:
                raise TestDiscoveryError(
                    "%s: %s is also defined by group %s" %
                    (where, name, ret.members[name][1]))
            function = value
            if isinstance(value, (staticmethod, classmethod)):
                function = value.__func__
            if name in super_users:
                raise TestDiscoveryError(
                    "%s: zero-argument super() does not work in a TestGroup; "
                    "name the suite class explicitly" % where)
            ret.members[name] = (value, group_name)
            info = getattr(function, "_autotest_testcase", None)
            if info is None:
                continue
            if not name.startswith(group_name):
                raise TestDiscoveryError(
                    "%s: test names must start with their group name (%s)" %
                    (where, group_name))
            if function.__doc__ is None:
                raise TestDiscoveryError("%s: test is missing a docstring" % where)
            ret.tests[name] = DiscoveredTest(name, function, info, group_dir)
            found_test = True
        if not found_test:
            raise TestDiscoveryError("%s defines no @testcase methods" % filepath)
    return ret


def test_root(root):
    '''return the (cached) TestRoot for a root directory'''
    if root not in _roots:
        _roots[root] = _collect(root)
    return _roots[root]


def root_owner(suite_class):
    '''return the class in suite_class's MRO declaring test_root, or None'''
    for klass in suite_class.__mro__:
        if klass.__dict__.get("test_root") is not None:
            return klass
    return None


def roots_for_class(suite_class):
    '''return the test roots visible to suite_class, nearest first'''
    ret = []
    for klass in suite_class.__mro__:
        root = klass.__dict__.get("test_root")
        if root is not None and root not in ret:
            ret.append(root)
    return ret


_installed = set()


def install(suite_class):
    '''install the methods of every group in the test roots of suite_class
    and its base classes onto the class declaring each root'''
    for klass in reversed(suite_class.__mro__):
        root = klass.__dict__.get("test_root")
        if root is None or klass in _installed:
            continue
        found = test_root(root)
        for (name, (value, group_name)) in found.members.items():
            if name in klass.__dict__:
                raise TestDiscoveryError(
                    "%s/%s defines %s, which %s already defines" %
                    (root, group_name, name, klass.__name__))
        for (name, (value, group_name)) in found.members.items():
            setattr(klass, name, value)
        _installed.add(klass)


def tests_for_bucket(suite_class, bucket):
    '''return the DiscoveredTests from suite_class's roots in bucket, by name'''
    ret = {}
    for root in reversed(roots_for_class(suite_class)):
        for t in test_root(root).tests.values():
            if bucket in t.info.buckets:
                ret[t.name] = t
    return [ret[name] for name in sorted(ret.keys())]


def find_test(suite_class, name):
    '''return the DiscoveredTest called name visible to suite_class, or None'''
    for root in roots_for_class(suite_class):
        t = test_root(root).tests.get(name)
        if t is not None:
            return t
    return None


def disabled_tests(suite_class):
    '''return {name: reason} for disabled tests visible to suite_class'''
    ret = {}
    for root in reversed(roots_for_class(suite_class)):
        for t in test_root(root).tests.values():
            if t.info.disabled is not None:
                ret[t.name] = t.info.disabled
    return ret


def check_buckets(bucket_classes):
    '''check every discovered test runs somewhere sensible

    bucket_classes maps a bucket name to the suite class which runs it.
    Each bucket a test names must exist and be run by the class owning
    the test's root, or by one of its subclasses.  Returns a list of
    problems found.'''
    problems = []
    owners = {}
    for suite_class in bucket_classes.values():
        for klass in suite_class.__mro__:
            root = klass.__dict__.get("test_root")
            if root is not None:
                owners[root] = klass
    for (root, owner) in sorted(owners.items(), key=lambda x: x[0]):
        for t in test_root(root).tests.values():
            for bucket in t.info.buckets:
                suite_class = bucket_classes.get(bucket)
                if suite_class is None:
                    problems.append("%s%s: unknown bucket %s" % (t.group_dir, t.name, bucket))
                elif not issubclass(suite_class, owner):
                    problems.append("%s%s: bucket %s is run by %s, which cannot run tests from %s" %
                                    (t.group_dir, t.name, bucket, suite_class.__name__, root))
    return problems
