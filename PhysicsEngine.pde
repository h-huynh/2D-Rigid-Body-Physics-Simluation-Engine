/*
A project by Henry Huynh
huynh407@umn.edu || henryh1404@gmail.com

A Processing implementation of an impulse-based 2D rigid body physics engine.

Major credit to Randy Gaul for his tutorials & ImpulseEngine source code.
Much of the functionality implemented is an adaptation of his C++ physics engine.
Impulse engine at GitHub: https://github.com/RandyGaul/ImpulseEngine
"How to Create a Custom Physics Engine": 
https://gamedevelopment.tutsplus.com/series/how-to-create-a-custom-physics-engine--gamedev-12715
*/

ArrayList<Body> bodyList = new ArrayList<Body>();
ArrayList<Manifold> manifolds = new ArrayList<Manifold>();

PVector gravity = new PVector(0,9.8);
long previousTime;
boolean paused = false;

void setup() {
  size(1280,720);
  noFill();
  restart();
  previousTime = millis(); //milliseconds since starting the program
  println("Welcome. Controls: ");
  println("press 1 for scene one (three cases), 2 for (dominoes), 3 for (circles), 4 for (polygons), g for gravity, and space for pause/unpause");
  println("left click for a random polygon, and right click for a random circle, r to restart");
}

//restart simulation
void restart() {
  gravity.set(0,0);
  bodyList.clear();
  setBoundaries();
  paused = false;
}

//update physical simulation
void update(float dt) { 
  for( Body b : bodyList ) {
    b.integrate(dt);
  }
  
  for( Manifold m : manifolds ) {
    m.applyImpulse();
    m.fixPenetration();
  }
  
  for( Body b : bodyList ) {
    b.force.set(0,0,0);
    b.torque.set(0,0,0);
  }
}

//check for key input
void keyReleased() {
  println("input: "+ key);
  switch(key) {
    case ' ':  paused =! paused; break;
    case 'g':  gravity.y = ( gravity.y == 0 ? 9.8 : 0 ); break;
    case 'r':  restart(); break;
    case 'n':  println(bodyList.size()); break;
    case '1':  scene1(); break;
    case '2':  scene2(); break;
    case '3':  scene3(); break;
    case '4':  scene4(); break;
    default: break;
  }
}

void mousePressed() {
  if (mouseButton == LEFT) {
    Polygon p = createPolygon(random(25,30), random(25,30),1);
    p.position = new PVector(mouseX,mouseY);
  } else { //right click
    Circle c = createCircle(random(5,25),1);
    c.position = new PVector(mouseX,mouseY);
  } 
}


void draw() {
  //increment timestep
  long now = millis();
  float dt = now - previousTime;
  previousTime = now;
  
  //check if simulation is paused
  if(paused) {
    dt=0;
  } else {
    dt*=0.001;
  }
  
  //draw
  background(0,0,0);
  pushMatrix();
  
  //clear manifolds
  manifolds.clear();
  
  //generate manifolds & handle collisions
  collisionHandler();
  
  //update simulation
  if(dt!=0) {
    update(dt);
  }
  
  //render all bodies
  for(Body b : bodyList) {
    b.render();
  }
 
  popMatrix();
}

//additional functions//------------------------------------------

//creating the bodies//

Circle createCircle(float radius,float mass) {
  Circle cir = new Circle(radius, PI * sq(radius) * mass);
  bodyList.add(cir);
  return cir;
}

Polygon createRectangle(float w, float h, float density) {
  Polygon rect = new Polygon();
  bodyList.add(rect);
  
  //create the four vertices
  rect.localPointList.add(new PVector(-w/2,-h/2));
  rect.localPointList.add(new PVector( w/2,-h/2));
  rect.localPointList.add(new PVector( w/2, h/2));
  rect.localPointList.add(new PVector(-w/2, h/2));
  
  //updates radius, shape, and computes the correct mass
  rect.computeDetails(density);
  return rect;
}

//a little helper function for creating boundaries (or platforms at a specified location)
Polygon createRectangleAtPosition(PVector a, PVector b) {
  Polygon rect = createRectangle(b.x-a.x, b.y-a.y, 1);
  rect.position.set((a.x + b.x)/2, (a.y + b.y)/2);
  return rect;
}

//random polygon of 4-10 sides
//has to be convex or SAT does not work
Polygon createPolygon(float min,float max,float density) {
  Polygon poly = new Polygon();
  bodyList.add(poly);
  
  //adjust num sides random(4,__) for more varying polygons
  int numSides = (int) random(4,10);
  for(int i=0; i < numSides; i++) {
    float radius = PI * 2 * (float)i / (float)numSides;
    float d = random(min,max);
    poly.localPointList.add(new PVector(cos(radius)*d,sin(radius)*d));
  }
  poly.computeDetails(density);
  return poly;
}

void randomPolygon(){
  Polygon p = createPolygon(random(25,30), random(25,30),1);
  p.position.set(random(width-p.maxRadius*2) + p.maxRadius,
                 random(height-p.maxRadius*2) + p.maxRadius);
}

void randomCircle(){
  Circle c = createCircle(random(5,25),1);
  c.position.set(random(width-c.radius*2) + c.radius,
                 random(height-c.radius*2) + c.radius);
}

//set the boundaries as static objects
void setBoundaries() {
  //top boundary
  Polygon b = createRectangleAtPosition(new PVector(0,0),new PVector(width,1)); //increase the 1 for thicker boundary
  b.setInfiniteMass();
  b.shapeColor = color(255,255,255);
  
  //left edge
  b = createRectangleAtPosition(new PVector(0,0),new PVector(1,height));
  b.setInfiniteMass();
  b.shapeColor = color(255,255,255);
  
  //right edge
  b = createRectangleAtPosition(new PVector(width-1,0),new PVector(width,height));
  b.setInfiniteMass();
  b.shapeColor = color(255,255,255);
  
  //bottom edge
  b = createRectangleAtPosition(new PVector(0,height-1),new PVector(width,height));
  b.setInfiniteMass();
  b.shapeColor = color(255,255,255);
  
}

//Scenes//---------------------------------------------------

void scene1(){
  println("Scene one: three collision cases");
  restart();
  
  //circle vs circle
  Circle c1 = createCircle(60,1);
  c1.position.set(width/6,height/6);
  c1.velocity.set(70,0);
  
  Circle c2 = createCircle(60,1);
  c2.position.set(2*width/6,height/6);
  
  //circle vs polygon
  Circle c3 = createCircle(45,1);
  c3.position.set(width/6,2.5*height/6);
  c3.velocity.set(70,0);
  
  Polygon r1 = createRectangle(80,160,10);
  r1.position.set(3*width/6,3*height/6);
  
  //polygon vs polygon
  Polygon r2 = createRectangle(80,160,10);
  r2.position.set(width/6,5*height/6);
  r2.angularVelocity.set(0,0,radians(10));
  r2.velocity.set(70,0);

  Polygon r3 = createRectangle(80,160,10);
  r3.position.set(4*width/6,5*height/6);
}

void scene2(){
  println("Scene two: dominoes");
  restart();
  gravity.set(0,9.8);
  
  Circle c1 = createCircle(45,1);
  c1.position.set(width/6,height-240);
  c1.velocity.set(75,0);
  
  Polygon r2 = createRectangle(40,240,10);
  r2.position.set(2*width/8,height-120);
  
  Polygon r3 = createRectangle(40,240,10);
  r3.position.set(3*width/8,height-120);
  
  Polygon r4 = createRectangle(40,240,10);
  r4.position.set(4*width/8,height-120);
  
  Polygon r5 = createRectangle(40,240,10);
  r5.position.set(5*width/8,height-120);
  
  Polygon r6 = createRectangle(40,240,10);
  r6.position.set(6*width/8,height-120);
  
  Polygon r7 = createRectangle(40,240,10);
  r7.position.set(7*width/8,height-120);
}

void scene3(){
  println("Scene three: testing circle overlap, reverse gravity");
  restart();
  gravity.set(0,-9.8);
  for(int i = 0; i < 150; i++){
    randomCircle();
  }
}

void scene4(){
  println("Scene three: testing polygon overlap, no gravity");
  restart();

  for(int i = 0; i < 40; i++){
    randomPolygon();             
    Body b = bodyList.get(bodyList.size()-1); 
    b.velocity.set(random(-40,40),random(-40,40));
    b.angularVelocity.z=random(-2,2);
  }
  
}
