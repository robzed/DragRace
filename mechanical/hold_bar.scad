// all dimensions are in mm
bar1_width = 12;    // mm
bar1_length = 133;
bar1_height = 2;

board_inner_edge_to_edge = 63.5;
front_edge_to_front_edge = board_inner_edge_to_edge-bar1_width;

module holes() {
    // holes
    union() {
        translate([5,10,5]) {
            cylinder(20, 3, 3, center=true, $fn=20);
        }
        translate([board_inner_edge_to_edge-5,10,5]) {
            cylinder(20, 3, 3, center=true, $fn=20);
        }
        translate([0,18,0]) {
            translate([5,10,5]) {
                cylinder(20, 3, 3, center=true, $fn=20);
            }
            translate([board_inner_edge_to_edge-5,10,5]) {
                cylinder(20, 3, 3, center=true, $fn=20);
            }
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

        cross_bar3_offset = 49;
        cross_bar3_width = 19;
        translate([0,cross_bar3_offset,0]) {
         cube([board_inner_edge_to_edge, cross_bar3_width, bar1_height], center = false);
        }
    }


    holes();
    translate([0,80,0]) holes();
}
