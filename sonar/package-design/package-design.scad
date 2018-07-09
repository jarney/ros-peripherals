$fn=100;


sensor_holder_width=45.5;
sensor_holder_hole=30;

///////////////////////////////////////////

// Un-comment the one you want to render.

// Object can be printed in 4 pieces with left/right
// glued together.
//upper_left_manifold();
//upper_right_manifold();
//lower_left_manifold();
//lower_right_manifold();

// Object can be printed in 2 pieces (top/bottom)
// provided that your print bed is big enough and you
// don't have first-layer adhesion problems.
//upper_manifold();
lower_manifold();

// Sensor holders are the pieces that
// the HC-SD04 modules fit into.
//sonar_sensor_holders();

// Full object.  Usually not 3d printable, but shown for reference.
//sonar_manifold();


///////////////////////////////////////////

module sonar_sensor_holders() {
    translate([0,0,0]) sonar_sensor_holder();
    translate([54,0,0]) sonar_sensor_holder();
    translate([54,54,0]) sonar_sensor_holder();
    translate([0,54,0]) sonar_sensor_holder();
    translate([0,110,0]) sonar_sensor_holder();
}


module sonar_sensor_holder(holes=true) {
    difference() {
        union() {
            if (!holes) {
                translate([0,0,5/2]) cube([45,45.5,5], center=true);
                translate([0,0,1]) cube([50.5,50,2], center=true);
                translate([0,0,100]) cube([46,45.5,200], center=true);
            }
            else {
                translate([0,0,5/2]) cube([46,44.5,5], center=true);
                translate([0,0,1]) cube([50.5,50,2], center=true);
            }
        }
        if (holes) {
            translate([11,5+8,0]) cylinder(r1=8.5, r2=8.3, h=30, center=true);
            translate([11,-5-8,0]) cylinder(r1=8.5, r2=8.3, h=30, center=true);
        }
    }
    
}

//sonar_sensor_holder(holes=false);
module sonar_manifold() {
    difference() {
        union() {
            difference() {
                union() {
                    cube([200,50,69]);
                    translate([75,50,0]) rotate([10,0,0]) translate([0,0,-30]) cube([50,50,90]);
                    translate([25,50,0]) rotate([10,0,45]) translate([0,-50,-20]) cube([50,90,90]);
                    translate([125,80,0]) rotate([10,0,-45]) translate([0,-50,-20]) cube([50,90,90]);
                }
                difference() {
                    union() {
                        translate([-10,2.5,2.5]) cube([220,45.5,45]);
                        translate([75,50,0]) rotate([10,0,0]) {
                            translate([45/2+2.5,50+2.5+0,45/2+2.5]) rotate([90,0,0]) sonar_sensor_holder(holes=false);
                        }
                        translate([25,50,0]) rotate([10,0,45]) {
                            translate([45/2+2.5,50+2.5,45/2+2.5]) rotate([90,0,0]) sonar_sensor_holder(holes=false);
                        }
                        translate([125,80,0]) rotate([10,0,-45]) {
                            translate([45/2+2.5,50+2.5,45/2+2.5]) rotate([90,0,0]) sonar_sensor_holder(holes=false);
                        }
                    }
                    translate([-100,-200,-85]) cube([400,400,90]);
                    translate([-100,-400+2.5,0]) cube([400,400,90]);
                }
                    
            }
            translate([50, 25, 0]) screw_post();
            translate([150, 25, 0]) screw_post();
            
            // Cradle for arduino controller.
            translate([110, 25-10, 5]) cube([4,4,50]);
            //translate([110, 25-22, 5]) cube([2,2,50]);
            translate([190, 25-10, 5]) cube([4,4,50]);
            //translate([190, 25-22, 5]) cube([2,2,50]);
            
            translate([110, 25-22, 5]) cube([2,12,10]);
            translate([190, 25-22, 5]) cube([2,12,10]);
            translate([110, 25-22, 5]) cube([80,2,10]);
            translate([110, 25-10, 5]) cube([80,2,10]);
            
        }
    
        translate([40,-10,24.9]) cube([6,20,6]);
        translate([-100,-200,-89.9]) cube([400,400,90]);
        translate([-100,-200,45+12]) cube([400,400,90]);
        translate([50, 25, 0]) screw_throughhole();
        translate([150, 25, 0]) screw_throughhole();
    }        

    translate([0,-20,45+7]) {
        difference() {
            cube([100,20,5]);
            translate([50-(83/2),20-12,-10]) cylinder(r1=1.8,r2=1.8,h=20);
            translate([50+83/2,20-12,-10]) cylinder(r1=1.8,r2=1.8,h=20);
        }
    }
    
    
}

module screw_post() {
        translate([0,0,-10]) cylinder(r1=5.5, r2=5.5, h=80);
}
module screw_throughhole() {
    translate([0,0,-11]) cylinder(r1=1.8, r2=1.8, h=82);
    translate([0,0,-58]) cylinder(r1=4, r2=4, h=82);
    translate([0,0,27]) cylinder(r1=3, r2=3, h=82);
}


module seam() {
            difference() {
            cube([8,8,5]);
            translate([4, 4, -5/2]) cylinder(r1=1.5, r2=1.5, h=10);
        }
    }

module lower_manifold() {
    difference() {
        sonar_manifold();
        upper_cutting_plane(0.2);
    }
}
module upper_manifold() {
    translate([-3,0,57]) 
        rotate([0,180,0]) {
            intersection() {
                sonar_manifold();
                upper_cutting_plane(-0.2);
            }
        }
}


module lower_left_manifold() {
    difference() {
        intersection() {
                sonar_manifold();
                translate([-300,-200,-20]) cube([400,400,400]);
        }
        upper_cutting_plane(0.2);
    }
}

module lower_right_manifold() {
    difference() {
        difference() {
            sonar_manifold();
            translate([-300,-200,-20]) cube([400,400,400]);
        }
        upper_cutting_plane(0.2);
        
    }
}

module upper_left_manifold() {
    rotate([0,180,0]) {
        intersection() {
            intersection() {
                sonar_manifold();
                translate([-300,-200,-20]) cube([400,400,400]);
            }
            upper_cutting_plane(-0.2);
        }
    }
}

module upper_right_manifold() {
    rotate([0,180,0]) {
        intersection() {
            difference() {
                sonar_manifold();
                translate([-300,-200,-20]) cube([400,400,400]);
            }
            upper_cutting_plane(-0.2);
        }
    }
}

module upper_cutting_plane(slop) {
    union() {
        translate([-100,-200,25]) cube([400,400,400]);
        
        translate([100,-0.01,23]) cube([20,1+slop,2.01]);
        translate([180,-0.01,23]) cube([20,1+slop,2.01]);
        translate([60,-0.01,23]) cube([20,1+slop,2.01]);
        translate([0,-0.01,23]) cube([20,1+slop,2.01]);
        
        translate([180+0.05,50-(1+slop),23]) cube([20,1+slop+0.01,2.01]);
        translate([0-0.05,50-(1+slop)+0.02,23]) cube([20,1+slop,2.01]);
    }
}


