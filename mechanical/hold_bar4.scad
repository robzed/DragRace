// v3 of hold bar

// all dimensions are in mm
bar1_width = 12;    // mm
bar1_length = 129;      // was 133
bar1_height = 5;        // test at 1mm, final at 5mm

board_inner_edge_to_edge = 63.5;
front_edge_to_front_edge = board_inner_edge_to_edge-bar1_width;

hole_r = 3/2;       // 3mm diameter

// middle cross bar
cross_bar3_offset = 69; // offset down
cross_bar3_width = 24;
first_bar3_hole_offset = 73;

module holes() {
    // holes
    union() {
        translate([4.5,7,5]) {
            cylinder(20, hole_r, hole_r, center=true, $fn=20);
        }
        translate([board_inner_edge_to_edge-4,7,5]) {
            cylinder(20, hole_r, hole_r, center=true, $fn=20);
        }
        translate([0,18,0]) {
            translate([4.5,7,5]) {
                cylinder(20, hole_r, hole_r, center=true, $fn=20);
            }
            translate([board_inner_edge_to_edge-4,7,5]) {
                cylinder(20, hole_r, hole_r, center=true, $fn=20);
            }
        }
    }
}
module secondary_holes() {
    union() {
        translate([8,55,5]) {
            cylinder(20, hole_r, hole_r, center=true, $fn=20);
        }
        translate([board_inner_edge_to_edge-7,55,5]) {
            cylinder(20, hole_r, hole_r, center=true, $fn=20);
        }
    }
}

difference() {
    union() {
        // front to back bars
        cube([bar1_width,bar1_length,bar1_height], center=false);

        translate([front_edge_to_front_edge, 0, 0]) {
            cube([bar1_width,bar1_length,bar1_height], center=false);
        }


        cross_bar1_offset = 10;
        cross_bar1_width = 10;

        cross_bar2_width = 10;
        cross_bar2_offset = (bar1_length-10)-cross_bar2_width;

        // cross bars
        translate([0,cross_bar1_offset,0]) {
         cube([board_inner_edge_to_edge, 10, bar1_height], center = false);
        }
        translate([0,cross_bar2_offset,0]) {
         cube([board_inner_edge_to_edge, cross_bar2_width, bar1_height], center = false);
        }

        // middle cross bar
        translate([0,cross_bar3_offset,0]) {
         cube([board_inner_edge_to_edge, cross_bar3_width, bar1_height], center = false);
        }
        

    }

    // remove holes and stuff
    union() {
        // holes on side posts
        holes();
        translate([0,81+18,0]) holes();
        // holes for sensors normally - just in case
        secondary_holes();
        translate([0,10,0]) secondary_holes();
        
        // remove corner for component
        translate([bar1_width,bar1_length,0]) rotate([0,0,45]) cube([8,8,bar1_height*4],center=true);        
        // holes on middle bar
        translate([board_inner_edge_to_edge/2,first_bar3_hole_offset,5]) {
            cylinder(20, hole_r, hole_r, center=true, $fn=20);
            translate([0,12.75,0]) {
                cylinder(20, hole_r, hole_r, center=true, $fn=20);
            }
        }
        
        // remove location for back driver
        // 30mm to 52mm
        translate([front_edge_to_front_edge-2,30,-2]) cube([21,21,bar1_height*4],center=false);
        

    }
}
