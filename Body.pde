abstract class Body{
  
  //Preset body variables----------------------------
  color shapeColor = color(255,255,255);
  
  //important variables to adjust for simulation realism
  float restitution = 0.5;
  float staticFriction = 0.75; //friction when not moving
  float dynamicFriction = 0.5; //friction in dynamic collisions
 
  //kinematic & dynamic variables---------------------
  
  PVector position = new PVector(0,0);
  PVector velocity = new PVector(0,0);
  PVector force = new PVector(0,0);
  
  PVector angle = new PVector(0,0,0);
  PVector angularVelocity = new PVector(0,0,0);
  PVector torque = new PVector(0,0,0);
  
  private float mass = 1;
  private float invMass = 1; // inverse, otherwise 1/mass
  private float momentInertia = 1; //moment of inertia
  private float invMomentInertia = 1; // inverse, otherwise 1/momentInertia
  
  //functions------------------------------------------
  
  //Each of the shapes have their own render functions
  abstract public void render();
  
  //mass related functions//
  
  float getMass() {
    return this.mass;
  }
  
  float getInverseMass() {
    return this.invMass;
  }
  
  void setMass(float newMass) {
    this.mass=newMass;
    if (newMass==0) //set inverse and prevent divide by 0
      this.invMass=0;
    else 
      this.invMass = 1.0 / newMass;
  }
  
  //inertia related functions//
  
  float getMomentOfInertia() {
    return this.momentInertia;
  }
  
  float getInverseMomentOfInertia() {
    return this.invMomentInertia;
  }
  
  void setMomentOfInertia(float newInertia) {
    this.momentInertia = newInertia;
    if(newInertia==0) //set inverse and prevent divide by 0
      this.invMomentInertia = 0;
    else 
      this.invMomentInertia = 1.0 / newInertia;
  }
  
 
  void setInfiniteMass() //use this when creating a static unmoving object
  {
    setMass(0);
    setMomentOfInertia(0);
  }
  
  //simple euler integration
  void integrate(float dt){
    integrateForces(dt);
    integrateVelocity(dt);
  }
  
  void integrateForces(float dt) {
    if(this.getInverseMass()==0) return;
    PVector f = PVector.add(gravity,PVector.mult(this.force,getInverseMass()));
    
    this.velocity.add( f.mult(dt/2.0) );
    this.angularVelocity.add( PVector.mult( this.torque, getInverseMomentOfInertia()*dt/2.0) );
  }
  
  void integrateVelocity(float dt) {
    this.position.add(PVector.mult(this.velocity,dt));
    this.angle.add(PVector.mult(this.angularVelocity,dt));
    integrateForces(dt);
  }
  
  
  //manifold related functions//
  
  // On contact, apply an impulse to body based on the contact vector.
  void contactImpulse(PVector contact, PVector impulse) {
    PVector linearVelocity = PVector.mult(impulse, getInverseMass());
    this.velocity.add(linearVelocity);
    this.angularVelocity.add(PVector.mult(contact.cross(impulse), getInverseMomentOfInertia()));
  }
  
  // returns distance to input pos
  PVector getDistance(PVector pos){
    return PVector.sub(pos, this.position);
  }
  
  // returns the sum of velocity and the cross of angularVelocity with the distance vector to input position.
  PVector netVelocityAtPosition(PVector pos){
    PVector distance = getDistance(pos);
    return PVector.add(this.velocity, this.angularVelocity.cross(distance));
  }
  
}//body
  
