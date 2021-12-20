class Polygon extends Body {
  float maxRadius;
  
  //list containing all the relevant positions of the important points for the polygon
  ArrayList<PVector> localPointList = new ArrayList<PVector>();
  
  
  //setup related functions//------------------------------------------
  
  //finds the max radius of the polygon based on its localPointList
  void findMaxRadius(){
    float newMaxRadius = -Float.MAX_VALUE;
    for(PVector point: localPointList){
      float radius = sqrt(sq(point.x-position.x) + sq(point.y-position.y));
      if (radius > newMaxRadius)
        newMaxRadius = radius;
    }
    maxRadius = newMaxRadius;
  }
  
   public void updateRadius() {
    float maxR = -Float.MAX_VALUE;
    for( PVector p : localPointList ) {
      float r = sqrt(sq(p.x-position.x) + sq(p.y-position.y));
      if(r>maxR) maxR=r;
    }
    maxRadius = maxR;
  }
  //helper function needed for calculating the mass of a polygon
  float calculateTriangleArea(PVector a, PVector b, PVector c) {
    return abs( (a.x*(b.y-c.y) + b.x*(c.y-a.y) + c.x*(a.y-b.y)) / 2.0 );
  }
  
  //function used for calculating & setting mass/moi when generating polygons of specified density
  void calculateMass(float density){
    int numPoints = localPointList.size();
    PVector pointOne = localPointList.get(0);
    PVector centroid = new PVector(0,0,0);
    PVector finalCentroid = new PVector(0,0,0);
    float area = 0;
    float moi = 0;
    
    //a polygon has a minimum of 3 points
    if (numPoints < 3) setMass(0);  
    
    //loop through points, calculating total area (using triangles), center of mass, and moment of inertia
    for(int i = 0; i < numPoints; i++) {
      int j = (i+1) % numPoints;
      
      //Use original point, iterated point, and center of mass to form a triangle. Add area to total area.
      PVector pointTwo = localPointList.get(j);
      float triangleArea = calculateTriangleArea(pointOne, pointTwo, centroid); 
      area += triangleArea;
      
      //calculating the new centroid and moment of inertia
      
      //centroid
      PVector adjustedCentroid = PVector.mult(PVector.add(centroid,PVector.add(pointOne,pointTwo)), 1.0/3.0);
      finalCentroid.add(adjustedCentroid);
      
      //moment of inertia
      float d = triangleArea*2;
      float intx2 = pointOne.x * pointOne.x + pointTwo.x * pointOne.x + pointTwo.x * pointTwo.x;
      float inty2 = pointOne.y * pointTwo.y + pointTwo.y * pointOne.y + pointTwo.y * pointTwo.y;
      moi += d * (0.25 / 3.0) * (intx2 + inty2);
      
      //set pointOne for the next triangle
      pointOne = pointTwo;
    }
    
    //finally, set the correct mass and moment of inertia
    setMass(density * area);
    setMomentOfInertia(density * moi);
  }
  
  //for updating the initial moment of inertia and localPointList based on the centroid of the polygon
  void setPolygon() {
    float area = 0;
    float moi = 0;
    PVector centroid = new PVector(0,0);
    
    for(int i = 0; i < localPointList.size(); i++) {
      PVector pointOne = localPointList.get(i);
      PVector pointTwo = localPointList.get((i+1) % localPointList.size());
      
      //area
      float d = pointOne.cross(pointTwo).z;
      float triangleArea = d / 2.0;
      area += triangleArea;
      
      //centroid
      centroid.add(PVector.add(pointOne, pointTwo).mult( (1.0/3.0) * triangleArea));
      
      //moment of inertia
      float intX2 = pointOne.x * pointOne.x + pointTwo.x * pointOne.x + pointTwo.x * pointTwo.x;
      float intY2 = pointOne.y * pointOne.y + pointTwo.y * pointOne.y + pointTwo.y * pointTwo.y;
      moi += (0.25 * (1.0/3.0) * d) * (intX2 + intY2);
    }
    
    //set the new adjusted moment of inertia
    setMomentOfInertia(moi);
    
    //adjust point list
    centroid.mult(1.0/area);
    for(PVector point: localPointList){
      point.sub(centroid);
    }
  }
  
  //composite function for the setup of a polygon
  void computeDetails(float density){
    findMaxRadius();
    setPolygon();
    calculateMass(density);
  }
  
  //collision and rendering related functions//------------------------
  
  //converts a point list from local to world frame
  PVector [] convertLocalToWorld() {
    PVector[] worldPointList = new PVector[localPointList.size()];
    for (int i = 0; i < localPointList.size(); i++){
      PVector worldPoint = (localPointList.get(i)).copy();
      worldPoint.rotate(this.angle.z);
      worldPoint.add(this.position);
      worldPointList[i] = worldPoint;
    }
    return worldPointList;
  }
  
  //finds the extreme point along a direction within a polygon, used for finding axis of least penetration
  PVector getSupport(PVector direction) {
    float bestProjection = -Float.MAX_VALUE;
    PVector[] points = convertLocalToWorld();
    PVector extremePoint = points[0];
    
    //iterating through points to find extreme point along input direction
    for(int i=0; i<points.length; ++i) {
      PVector current = points[i];
      float projection = current.dot(direction);

      if(projection > bestProjection) {
        extremePoint = current;
        bestProjection = projection;
      }
    }
    
    return extremePoint;
  }
  
  
  void render(){
    PVector[] vertices = convertLocalToWorld();
    int numVertices = vertices.length;
    
    fill(shapeColor);
    stroke(shapeColor);
    
    beginShape();
    
    //set vertices
    for(int i = 0; i < numVertices; i++){
      vertex(vertices[i].x,vertices[i].y);
    }
    endShape(CLOSE);
    //draw outlines
    
    stroke(0,0,0); //outline color
    for(int i = 0; i < numVertices; i++){
      int j = (i+1) % numVertices;
      line(vertices[i].x, vertices[i].y, vertices[j].x, vertices[j].y);
    }
  }
  
}//polygon
