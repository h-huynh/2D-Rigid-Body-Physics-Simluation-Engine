class Circle extends Body {
  float radius = 2; //default radius of 2
  
  //default constructor
  Circle() {
    super();
  }
  
  //constructor with specified radius and mass
  Circle(float newRadius, float newMass){
    this.radius = newRadius;
    this.setMass(newMass);
    this.setMomentOfInertia(newMass * sq(radius) / 4.0); // moment of inertia of a thin disk  
  }
  
  //draws the circle
  void render(){
    fill(shapeColor);
    stroke(0,0,0); //outline color
    circle(position.x, position.y, 2*radius);
  }
  
}//circle
