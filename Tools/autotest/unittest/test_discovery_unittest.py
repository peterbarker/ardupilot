#!/usr/bin/env python3

'''
Unit tests for Tools/autotest/test_discovery.py

AP_FLAKE8_CLEAN
'''

import os
import tempfile
import textwrap
import unittest

import test_discovery

HEADER = "from test_discovery import TestGroup, testcase\n\n\n"


class TestDiscoveryTest(unittest.TestCase):

    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.root = self.tmp.name

    def tearDown(self):
        self.tmp.cleanup()

    def add_group(self, name, body, root=None):
        if root is None:
            root = self.root
        group_dir = os.path.join(root, name)
        os.makedirs(group_dir, exist_ok=True)
        with open(os.path.join(group_dir, name + ".py"), "w") as f:
            f.write(HEADER + textwrap.dedent(body))
        return group_dir

    def suite_class(self, root=None, **attributes):
        if root is None:
            root = self.root
        attributes["test_root"] = root
        return type("Suite", (object,), attributes)

    def test_family_and_helpers_installed(self):
        self.add_group("Deadreckoning", '''
            class Deadreckoning(TestGroup):
                def deadreckoning_main(self, no_airspeed=False):
                    return ("main", no_airspeed)

                @testcase(bucket="B1")
                def DeadreckoningNoAirSpeed(self):
                    \'\'\'no airspeed\'\'\'
                    return self.deadreckoning_main(no_airspeed=True)

                @testcase(bucket=["B1", "B2"], attempts=3, speedup=8)
                def Deadreckoning(self):
                    \'\'\'with airspeed\'\'\'
                    return self.deadreckoning_main()
            ''')
        # a resource directory without a test file is ignored:
        os.makedirs(os.path.join(self.root, "SomeMissionFiles"))
        suite = self.suite_class()
        test_discovery.install(suite)

        instance = suite()
        self.assertEqual(instance.DeadreckoningNoAirSpeed(), ("main", True))
        self.assertEqual(instance.Deadreckoning(), ("main", False))

        names = [t.name for t in test_discovery.tests_for_bucket(suite, "B1")]
        self.assertEqual(names, ["Deadreckoning", "DeadreckoningNoAirSpeed"])
        names = [t.name for t in test_discovery.tests_for_bucket(suite, "B2")]
        self.assertEqual(names, ["Deadreckoning"])

        t = test_discovery.find_test(suite, "Deadreckoning")
        self.assertEqual(t.info.attempts, 3)
        self.assertEqual(t.info.speedup, 8)
        self.assertEqual(t.group_dir, self.root + "/Deadreckoning/")
        t = test_discovery.find_test(suite, "DeadreckoningNoAirSpeed")
        self.assertEqual(t.group_dir, self.root + "/Deadreckoning/")
        self.assertIsNone(test_discovery.find_test(suite, "Missing"))

    def test_disabled(self):
        self.add_group("Flappy", '''
            class Flappy(TestGroup):
                @testcase(bucket="B1", disabled="Flapping")
                def Flappy(self):
                    \'\'\'flaps\'\'\'
                    pass
            ''')
        suite = self.suite_class()
        self.assertEqual(test_discovery.disabled_tests(suite), {"Flappy": "Flapping"})

    def test_subclass_root_overrides_and_sees_base_tests(self):
        base_root = os.path.join(self.root, "base")
        sub_root = os.path.join(self.root, "sub")
        self.add_group("AutoTune", '''
            class AutoTune(TestGroup):
                @testcase(bucket="Base")
                def AutoTune(self):
                    \'\'\'base autotune\'\'\'
                    return "base"
            ''', root=base_root)
        self.add_group("Shared", '''
            class Shared(TestGroup):
                @testcase(bucket=["Base", "Sub"])
                def Shared(self):
                    \'\'\'runs in both\'\'\'
                    return "shared"
            ''', root=base_root)
        self.add_group("AutoTune", '''
            class AutoTune(TestGroup):
                @testcase(bucket="Sub")
                def AutoTune(self):
                    \'\'\'sub autotune\'\'\'
                    return "sub"
            ''', root=sub_root)
        base = self.suite_class(root=base_root)
        sub = type("Sub", (base,), {"test_root": sub_root})
        test_discovery.install(sub)
        self.assertEqual(sub().AutoTune(), "sub")
        self.assertEqual(base().AutoTune(), "base")
        names = [t.name for t in test_discovery.tests_for_bucket(sub, "Sub")]
        self.assertEqual(names, ["AutoTune", "Shared"])
        self.assertEqual(test_discovery.roots_for_class(sub), [sub_root, base_root])
        self.assertEqual(test_discovery.find_test(sub, "AutoTune").group_dir, sub_root + "/AutoTune/")

    def assert_collect_fails(self, message):
        with self.assertRaisesRegex(test_discovery.TestDiscoveryError, message):
            test_discovery.test_root(self.root)

    def test_name_must_start_with_group(self):
        self.add_group("Group", '''
            class Group(TestGroup):
                @testcase(bucket="B1")
                def Other(self):
                    \'\'\'misnamed\'\'\'
                    pass
            ''')
        self.assert_collect_fails("must start with their group name")

    def test_docstring_required(self):
        self.add_group("Group", '''
            class Group(TestGroup):
                @testcase(bucket="B1")
                def Group(self):
                    pass
            ''')
        self.assert_collect_fails("missing a docstring")

    def test_class_must_match_file(self):
        self.add_group("Group", '''
            class Grop(TestGroup):
                @testcase(bucket="B1")
                def Group(self):
                    \'\'\'misnamed class\'\'\'
                    pass
            ''')
        self.assert_collect_fails("must define class Group")

    def test_group_needs_a_test(self):
        self.add_group("Group", '''
            class Group(TestGroup):
                def helper(self):
                    pass
            ''')
        self.assert_collect_fails("defines no @testcase")

    def test_duplicate_member(self):
        for name in "GroupA", "GroupB":
            self.add_group(name, '''
                class %s(TestGroup):
                    def helper(self):
                        pass

                    @testcase(bucket="B1")
                    def %s(self):
                        \'\'\'test\'\'\'
                        pass
                ''' % (name, name))
        self.assert_collect_fails("helper is also defined by group GroupA")

    def test_zero_argument_super(self):
        self.add_group("Group", '''
            class Group(TestGroup):
                @testcase(bucket="B1")
                def Group(self):
                    \'\'\'uses super\'\'\'
                    super().Group()
            ''')
        self.assert_collect_fails("zero-argument super")

    def test_explicit_super_allowed(self):
        self.add_group("Group", '''
            class Base(object):
                def Group(self):
                    return "base"

            class Group(TestGroup):
                @testcase(bucket="B1")
                def Group(self):
                    \'\'\'uses explicit super\'\'\'
                    class Nested(Base):
                        def Group(self):
                            return super().Group()
                    return super(Base, self).Group
            ''')
        self.assertIn("Group", test_discovery.test_root(self.root).tests)

    def test_empty_bucket_list(self):
        with self.assertRaises(test_discovery.TestDiscoveryError):
            test_discovery.testcase(bucket=[])

    def test_install_refuses_duplicate_of_class_method(self):
        self.add_group("Group", '''
            class Group(TestGroup):
                @testcase(bucket="B1")
                def Group(self):
                    \'\'\'test\'\'\'
                    pass
            ''')

        def Group(self):
            pass
        suite = self.suite_class(Group=Group)
        with self.assertRaisesRegex(test_discovery.TestDiscoveryError, "already defines"):
            test_discovery.install(suite)

    def test_check_buckets(self):
        base_root = os.path.join(self.root, "base")
        sub_root = os.path.join(self.root, "sub")
        self.add_group("Good", '''
            class Good(TestGroup):
                @testcase(bucket=["Base", "Sub"])
                def Good(self):
                    \'\'\'fine\'\'\'
                    pass
            ''', root=base_root)
        self.add_group("Unknown", '''
            class Unknown(TestGroup):
                @testcase(bucket="Nowhere")
                def Unknown(self):
                    \'\'\'unknown bucket\'\'\'
                    pass
            ''', root=base_root)
        self.add_group("WrongSuite", '''
            class WrongSuite(TestGroup):
                @testcase(bucket="Base")
                def WrongSuite(self):
                    \'\'\'the base suite cannot run sub tests\'\'\'
                    pass
            ''', root=sub_root)
        base = self.suite_class(root=base_root)
        sub = type("Sub", (base,), {"test_root": sub_root})
        problems = test_discovery.check_buckets({"Base": base, "Sub": sub})
        self.assertEqual(len(problems), 2, problems)
        self.assertIn("Unknown: unknown bucket Nowhere", problems[0])
        self.assertIn("WrongSuite: bucket Base is run by Suite", problems[1])


if __name__ == '__main__':
    unittest.main()
