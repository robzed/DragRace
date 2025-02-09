//import("pololu-gear-motor-bracket-standard.stl");
import("pololu-gear-motor-bracket-extended.stl");

// stop up top holes
//translate([2,-11.5,10]) cube([5,23,2]);

module pegs(y_offset)
{
    translate([4.25, y_offset, -2])
    {   
//        difference()
        {
            //cylinder(20, 1.5, 1.5, $fn=20);
            translate([9.5, 0, 5])
            {
                cylinder(20, 1.6, 1.6, $fn=30);
            }
        
            // cutout clips
            //translate([-5, -0.5, -0.01]) cube([10, 1, 2]);
        }
    }
}



    difference() 
    {
        translate([9.5, -13.5, 11]) {
            cube([8.5,27,2],center=false);
        }
        union()
        {
            pegs(8.9);
            pegs(-8.9);
        }
}