This is my rendition of the code as developed by Kristian Blasol (of Modular in a Week fame).
For the original software, a schematic and other hardware related info, have a look at: https://github.com/SourceryOne/Arduino-Sample-and-Hold

As I don't like the use of globals, to make the code a bit more readable and avoid (future) side effects I packed all of them in parameter calls to the original functions.
In my experience the code functions the same as Kristian's code.
From the various uses proposed for the MIN and MAX controls I liked the one that controls the speed of sliding up to or down from one output value to the next the most, so I implemented that variant.
