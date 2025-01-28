local simple_library = {}  -- this object is the "library"

-- add a data object to the library:
simple_library.simple_map = {
  ['greeting']='Yowsers!'
}

-- add a method to the library:
function simple_library.say_hi()
   print("Simple library saying hi")
end

-- return the library object
return simple_library
