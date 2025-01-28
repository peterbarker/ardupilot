-- simple script which uses a library to do things

simple_library = require("simple_library")

function update() -- this is the loop which periodically runs
  gcs:send_text(0, "simple script starting")

  -- retrieve data from a map in the library:
  gcs:send_text(0, "the library has greeting:" .. simple_library.simple_map["greeting"])
  -- call a method on the library:
  simple_library.say_hi()

  return update, 1000 -- reschedules the loop
end

return update() -- run immediately before starting to reschedule
