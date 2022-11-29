$fn = 100;

// Parametres des cerlces
rayon_ext = 9 ; // rayon extérieur des cercles
epaisseur_c = 2 ;
rayon_int = rayon_ext - epaisseur_c ; // rayon extérieur des cercles
hauteur_c = 2;

// paramètres des étages
hauteur_e = 4;
dec_centre = 11; // décalage des cercles par rapport au centre

// paramètres des tubes
hauteur_t = 44;
rayon_t = 8;
epaisseur_tube = 2;
rayon_t_int = rayon_t - epaisseur_tube;

tab_cercle = [3,9,15,21,27,33,39];

///// Pour tester les modules
//cercle(height = hauteur_c, radius_ext = rayon_ext, radius_int = rayon_int);
// cercle(height = 5, radius_ext = 3, radius_int = 2);
//etage(height=hauteur_e,radius=rayon_ext,radius_rect=10,shift=dec_centre);
//tube(height_tube=hauteur_t,height_circle=hauteur_c,tube_radius=rayon_t,tube_radius_int=rayon_t_int,radius_ext=rayon_ext,radius_int=rayon_int,circle_tab=tab_cercle);
//tritube_plein();

/* Parametres lors des tests
height = hauteur_e;
radius = rayon_ext;
radius_rect = 10;

height_tube = hauteur_t;
height_circle = hauteur_c;
tube_radius = rayon_t;
tube_radius_int = rayon_t_int;
radius_ext = rayon_ext;
radius_int = rayon_int;
*/

section();
translate([0,0,hauteur_t+3+hauteur_e])etage(height=hauteur_e,radius=rayon_ext,radius_rect=10,shift=dec_centre);
translate([0,0,hauteur_t+3+hauteur_e*2])section();

module section(){
tritube();

etage(height=hauteur_e,radius=rayon_ext,radius_rect=10,shift=dec_centre);
difference(){
    translate([0,0,21+3])etage(height=hauteur_c,radius=rayon_ext,radius_rect=10,shift=dec_centre);
        tritube_plein();
}
translate([0,0,hauteur_t+3])etage(height=hauteur_e,radius=rayon_ext,radius_rect=10,shift=dec_centre);
}

 module tritube_plein()
 {
     translate([0,0,3])union(){
        translate([dec_centre,0,0])tube(height_tube=hauteur_t,height_circle=hauteur_c,tube_radius=rayon_t,tube_radius_int=0,radius_ext=rayon_ext,radius_int=rayon_int,circle_tab=tab_cercle);
        rotate([0,0,120])translate([dec_centre,0,0])tube(height_tube=hauteur_t,height_circle=hauteur_c,tube_radius=rayon_t,tube_radius_int=0,radius_ext=rayon_ext,radius_int=rayon_int,circle_tab=tab_cercle);
        rotate([0,0,240])translate([dec_centre,0,0])tube(height_tube=hauteur_t,height_circle=hauteur_c,tube_radius=rayon_t,tube_radius_int=0,radius_ext=rayon_ext,radius_int=rayon_int,circle_tab=tab_cercle);
    }
}


 module tritube()
 {
     translate([0,0,3])union(){
        translate([dec_centre,0,0])tube(height_tube=hauteur_t,height_circle=hauteur_c,tube_radius=rayon_t,tube_radius_int=rayon_t_int,radius_ext=rayon_ext,radius_int=rayon_int,circle_tab=tab_cercle);
        rotate([0,0,120])translate([dec_centre,0,0])tube(height_tube=hauteur_t,height_circle=hauteur_c,tube_radius=rayon_t,tube_radius_int=rayon_t_int,radius_ext=rayon_ext,radius_int=rayon_int,circle_tab=tab_cercle);
        rotate([0,0,240])translate([dec_centre,0,0])tube(height_tube=hauteur_t,height_circle=hauteur_c,tube_radius=rayon_t,tube_radius_int=rayon_t_int,radius_ext=rayon_ext,radius_int=rayon_int,circle_tab=tab_cercle);
    }
}

module tube(height_tube,height_circle,tube_radius,tube_radius_int,radius_ext,radius_int,circle_tab)
{
    difference(){
        cylinder(height_tube,tube_radius,tube_radius);
        translate([0,0,-1])cylinder(height_tube+5,tube_radius_int,tube_radius_int);
            for ($repetitions = tab_cercle)
        {
            translate([0, 0, $repetitions])cercle(height = hauteur_c, radius_ext = rayon_ext, radius_int = rayon_int+0.5);
        }
    }
}

module etage(height,radius,radius_rect,shift)
{
    union(){
        translate([shift,0,0])cylinder(height,radius,radius);
        rotate([0,0,120])translate([shift,0,0])cylinder(height,radius,radius);
        rotate([0,0,240])translate([shift,0,0])cylinder(height,radius,radius);

        rotate([0,0,180])translate([shift/2-1,-radius_rect,0])cube(size = [radius_rect,radius_rect*2,height]);
        rotate([0,0,60])translate([shift/2-5,-radius_rect+5,0])cube(size = [radius_rect,radius_rect,height]);
        rotate([0,0,-60])translate([shift/2-5,-radius_rect+5,0])cube(size = [radius_rect,radius_rect,height]);
        translate([-radius_rect/2,-radius_rect/2,0])cube(size = [radius_rect,radius_rect,height]);
    }
}


module cercle(height,radius_ext,radius_int) 
{
    difference(){
    cylinder(height,radius_ext,radius_ext);
    translate([0,0,-1])cylinder(height+2,radius_int,radius_int);
    }
}