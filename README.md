# Implicit-Euler-Method-for-DAE-system

Run the following script to use the implicit Euler's method (i.e. the first-order BDF (backward-differential-formulism) method) for DAE system.

    main_Slider_Implicit.m

The test case is a slider that goes down a slope under the gravitational force, without any friction.

For comparative study, run the following script to see the result of the Baumgarte's method, which is one of the basic explicit solver for rigid body systems. 

    main_Slider.m

To be continued. The current method only takes acc constraints and velocity constraints into consideration. When adding the geometric constraints, I currently cannot obtain the correct results.
