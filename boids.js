
function Boid(dxx) {
  // constructor
  this.acceleration = createVector(0,0);
  let vw = 0.1;
  this.velocity = createVector(random(-vw,vw),random(-vw,vw));
  let xw = 130;
  this.position = createVector(random(-xw,xw),random(-xw,xw));
  this.phi = random(0,2*PI);
  this.phidot = 0.0;
  this.r = 5.0;  // for display and in pixels
  this.m = boid_mass;
  this.dx = dxx; // scale multiply by this to get pixels, used in display

}

// Lots of Boid stuff from p5 examples
Boid.prototype.render = function() {
  // Draw a triangle rotated in the direction of velocity
  var theta = this.velocity.heading() + radians(90);
  push();
  fill(0,300,100);
  stroke(20);
  strokeWeight(1);
  translate(0,0);
  translate(width/2+this.position.x*this.dx,height/2+this.position.y*this.dx);
  rotate(theta);
  beginShape();
  vertex(0, -this.r*2);
  vertex(-this.r, this.r*2);
  vertex(this.r, this.r*2);
  endShape(CLOSE);
  rotate(-theta + this.phi);
  rotate(this.phi);
  fill('rgb(50,100,250)');
  ellipse(0,0,6*this.r,this.r);
  pop();
      
}


// wrap boids in display, but note that forces are not yet wrapped.
Boid.prototype.borders = function(){
  //print(this.dx);
  let wreal = (width + this.r)/this.dx ;
  let hreal = (height+ this.r)/this.dx ;
  if (this.position.x > 0.5*wreal){
    this.position.x -= wreal;
  }
  if (this.position.x < -0.5*hreal){
    this.position.x += wreal;
  }
  if (this.position.y > 0.5*wreal){
    this.position.y -= hreal;
  }
  if (this.position.y < -0.5*hreal){
    this.position.y += hreal;
  }
}

// a flock of boids
function Flock(nboids) {
   // An array for all the boids
   this.boid_set = []; // Initialize the array of boids
   this.dx = dx;  // used in display
    
   this.boidspeed = boidspeed; // relevant for propulsion
   
   this.rsoft = 2.0;  // softening length
  
     // distances
   this.d_attract   = d_attract; // distance for attraction/repulsion forces between Boids
   this.d_repel   = d_repel;  // distance for attraction/repulsion forces between Boid
   this.d_align   = d_align;  // distance for alignment
   this.d_rotate =   d_rotate;  // for rotation force
   this.d_anglepush = d_anglepush;  // for anglepush
   this.d_vforce = d_vforce; // in vforce
  
   // strengths, units acceleration
   this.attract_force = attract_force;  // for attact force size
   this.repel_force   = repel_force;    // for repel force size
   this.align_force   = align_force;   // allign/cohesion 
   this.propel_force  = propel_force;  // for propel
   this.rotate_force = rotate_force;  // for rotation
   this.anglepush_force = anglepush_force; // for anglepush
   this.vforce_force = vforce_force;  // for vforce
    
  // create nboids of Boids!
  for (let i=0;i<nboids;i++){
     let b = new Boid(this.dx);
     this.boid_set.push(b);
  }
  
  this.display_flock = function(){  // display boids
    let n = this.boid_set.length;
    for (let i=0;i<n;i++){
      let b = this.boid_set[i];
      b.render();
    }
  }
  // call border wrap function
  this.borders = function(){
    let n = this.boid_set.length;
    for (let i=0;i<n;i++){
      let b = this.boid_set[i];
      b.borders();
    }
  }
  // zero all the accelerations
  this.zeroaccel = function(){
    let n = this.boid_set.length;
    for(let k = 0;k<n;k++){
      this.boid_set[k].acceleration.mult(0);
      this.boid_set[k].phidot = 0;
    }
  }
  
  // friction/dissipation force that depends on velocity differences for nearby boids
  this.vforce = function() {
    let n = this.boid_set.length;
    for (let i = 0; i < n-1; i++) {
      let bi = this.boid_set[i];
      for (let j = i+1; j <n; j++) {
        let bj = this.boid_set[j];
        let dr = p5.Vector.sub(bi.position,bj.position);
        let r_len = dr.mag();
        if (r_len < this.d_vforce){ // distance limit, not weighting by distance
          let dv = p5.Vector.sub(bi.velocity,bj.velocity);
          let Force = dv.copy();
          // Force.normalize();
          Force.mult(this.vforce_force);
          let ai = p5.Vector.mult(Force,-1/bi.m);
          let aj = p5.Vector.mult(Force, 1/bj.m);
          bi.acceleration.add(ai);
          bj.acceleration.add(aj);
        }
      }
    }
  }
  
  // Repel force depends on inverse distance -- all boid pairs
  // if you change the sign of repel_force then can be attractive
  this.repel = function() {
    let n = this.boid_set.length;
    for (let i = 0; i < n-1; i++) {
      let bi = this.boid_set[i];
      for (let j = i+1; j <n; j++) {
        let bj = this.boid_set[j];
        let dr = p5.Vector.sub(bi.position,bj.position);
        let r_len = dr.mag();  // length of interboid distance
        if (r_len < this.d_repel){ // distance cutoff
          let drhat = dr.copy();
          drhat = drhat.normalize();  // unit vector for direction
          let Force = drhat.copy();
          let fac = -1.0*this.repel_force*this.d_repel/(r_len + this.rsoft); // normalized here 
          // note use of softening 
          // if this is negative then it is repulsive, otherwise attractive
          // fac *= cos(2*(bi.phi - bj.phi)); 
          Force.mult(fac); 
          let ai = p5.Vector.mult(Force,-1/bi.m);
          let aj = p5.Vector.mult(Force, 1/bj.m);
          bi.acceleration.add(ai);
          bj.acceleration.add(aj);
        }
      // down side of this method is if we have many nearby particles, acceleration gets high
      }
    }
  }
  
  // trying out a new force!
  this.anglepush = function() {
    let n = this.boid_set.length;
    for (let i = 0; i < n-1; i++) {
      let bi = this.boid_set[i];
      for (let j = i+1; j <n; j++) {
        let bj = this.boid_set[j];
        let dr = p5.Vector.sub(bi.position,bj.position);
        let dv = p5.Vector.sub(bi.velocity,bj.velocity);
        let r_len = dr.mag();  // length of interboid distance
        if (r_len < this.d_anglepush){
          let drhat = dr.copy();
          drhat = drhat.normalize();  // unit vector for direction between particles
          // let Force = drhat.copy();
          let fac = 1.0*this.anglepush_force; // normalized here?
          let ang = (bi.phi + bj.phi)%(2*PI); // take ave, modulo and mult by 2 
          let Force = createVector(cos(ang),sin(ang)); 
          let ss = p5.Vector.dot(drhat,Force);  // check sign
          if (ss >0){
            Force.mult(-1);
          }
          // fac *= cos(2*(bi.phi - bj.phi)); // 
          Force.mult(fac); 
          let ai = p5.Vector.mult(Force,-1/bi.m);
          let aj = p5.Vector.mult(Force, 1/bj.m);
          bi.acceleration.add(ai);
          bj.acceleration.add(aj);
        }
      }
    }
  }
    
    // Cohesion force, doing all pairs
  this.cohesion = function() {
     if (this.attract_force >0){
        let n = this.boid_set.length;
        for (let i = 0; i < n; i++) {
           let bi = this.boid_set[i];
           let pos_sum = createVector(0,0);
           let count = 0;
           for (let j = 0; j < n; j++) {
              let bj = this.boid_set[j];
              let dr = p5.Vector.sub(bi.position,bj.position);
              let r_len = dr.mag();  // length of interboid distance
              if ((r_len > 0) && (r_len < this.d_attract)){
                  pos_sum.add(bj.position); // sum of positions
                  count++;
              }
            }
            if (count >0){
              pos_sum.div(count);   // is average of nearby boid positions!
              let Force = p5.Vector.sub(pos_sum, bi.position); // target direction
              Force.normalize();
              Force.mult(this.boidspeed);
              Force.sub(bi.velocity);
              let ai = p5.Vector.mult(Force,this.attract_force/bi.m);  
              bi.acceleration.add(ai);
            }
         }
     }
  }
  
  // alignment try to stear toward mean velocity of nearby Boids
  // velocity dependent forces here!
  this.align = function() {  
    let n = this.boid_set.length;
    // For every boid in the system, check if it's close to another
    for (let i = 0; i < n; i++) {
      let count = 0;
      let bi = this.boid_set[i];
      // let steer = createVector(0,0);  // from average of nearest neighbor velocities
      let v_ave = createVector(0,0);
      for (let j = 0; j < n; j++) {
        let bj = this.boid_set[j];
        let dr = p5.Vector.dist(bi.position,bj.position);
        // If the distance is greater than 0 and less than an arbitrary amount (0 when you are yourself)
        if ((dr > 0) && (dr < this.d_align)) {
          v_ave.add(bj.velocity);  // sum of velocities of nearby
          count++; // Keep track of how many nearby
        }
      }
      if (count > 0){
        v_ave.normalize();
        v_ave.mult(this.boidspeed);
        let steer = p5.Vector.sub(v_ave,bi.velocity);
        // desired is now the stear 
        // As long as the vector is greater than 0
        if (steer.mag() > 0) {
            steer.mult(this.align_force/bi.m);
            // steer.limit(this.maxforce);
            bi.acceleration.add(steer);//  only on bi
                }
        }
     }
  }
  
  // try to reach same velocity, for is propto propel_force times 
  // difference in velocity from boidspeed
  this.propel = function(){
    let n = this.boid_set.length;
    for (let i = 0; i < n; i++) {
      let bi = this.boid_set[i];
      let vi = bi.velocity.copy();
      let vmag = vi.mag();  // length
      let vhat = vi.copy(); // unit vector
      vhat.normalize();
      let Force = vhat.copy(); // direction same as velocity
      Force.mult((vmag-this.boidspeed)*this.propel_force);
      let ai = p5.Vector.div(Force,-bi.m);
      bi.acceleration.add(ai);
    }
  }
    
  this.rotate = function(){
    let n = this.boid_set.length;
    for (let i = 0; i < n; i++) {
       let count = 0;
       let bi = this.boid_set[i];
       // let steer = createVector(0,0);  // from average of nearest neighbor velocities
       let v_ave = createVector(0,0);
       for (let j = i; j < n-1; j++) {
         let bj = this.boid_set[j];
         let dr = p5.Vector.dist(bi.position,bj.position);
         if (dr<this.d_rotate){
             let rforce = sin(2*(bj.phi - bi.phi))*this.rotate_force;
             bi.phidot += rforce; // assuming all particles have the same moment of inertia
             bj.phidot -= rforce;
         }
       }
     }
   }
   
  // integrate dt
  this.single_timestep = function(dt){
    // this.zeroaccel();
    // this.propel(); // propel boids
    // this.repel(); // repel repulsion
    // this.cohesion(); // attraction repulsion
    // this.align();
    // boid_node_interact(boid_set,node_set,
    //        force_amp,force_k,vforce_amp); 
    // apply interactions between boids and nodes
    let n = this.boid_set.length;
    for (let i = 0; i < n; i++) {
      bi = this.boid_set[i];
      let dv = p5.Vector.mult(bi.acceleration,dt); // h
      bi.velocity.add(dv); // 
      let dr = p5.Vector.mult(bi.velocity,dt);
      bi.position.add(dr); //
      bi.phi += bi.phidot*dt;
    }
  }
    
  this.shift = function(centroid){
        let n = this.boid_set.length;
        for(let i = 0;i< n;i++){
            boidi = this.boid_set[i];
            boidi.position.sub(centroid);
        }
    }
  
}
