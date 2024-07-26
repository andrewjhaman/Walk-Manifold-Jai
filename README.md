# Walk Manifold Jai Implementation

A basic implementation of the Walk Manifold algorithm used in The Witness, written in JAI.
The algorithm takes in a list of triangles representing the "walkable" surface, as well as some primitives
representing "blockers" and return a connected manifold corresponding to valid positions for a specific "mover."

![walk_manifold_visualization](https://github.com/andrewjhaman/Walk-Manifold-Jai/blob/main/walk_manifold_01.gif?raw=true)

The algorithm itself generalizes to arbitrary functions for the surfaces and blockers. If for example you wanted
to feed in a heightmap instead of triangles, that could be done with minimal code changes.

An explanation of the algorithm by it's inventor Casey Muratori can be found here 
https://www.youtube.com/watch?v=YE8MVNMzpbo

At the moment I am providing this without changes from my engine, just as a reference for
other people looking to implement the algorithm. If you would find it useful as a module then shoot me a message, 
and if I get a few of those, I'll pull it out into a standalone thing.

The performance is pretty decent enough for my purposes for now, but there is still some low-hanging fruit performance wise.
Some of these are marked with //@Speed comments, others are not.
