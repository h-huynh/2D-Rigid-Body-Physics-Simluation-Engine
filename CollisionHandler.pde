//simple class to pass around info needed to handle collisions having to do with polygons
class Vertex {
  int idx = 0;
  float val = -Float.MAX_VALUE;
}

//collision helper functions//--------------------------------

PVector getNormalTo(PVector start, PVector end){
  PVector n = PVector.sub(end,start);
  float temp = n.x;
  n.x=n.y;
  n.y=-temp;
  n.normalize();
  return n;
}

//separating axis theorem related functions
Vertex findAxisOfLeastPenetration(Polygon a, Polygon b){
  Vertex leastVertex = new Vertex();
  PVector[] vertices = a.convertLocalToWorld();
  int numVertices = vertices.length;
  
  for(int i = 0; i < numVertices; i++){
    int j = (i+1) % numVertices;
    PVector normal = getNormalTo(vertices[i],vertices[j]);
    PVector support = b.getSupport(PVector.mult(normal,-1));
    
    float penetrationDistance = normal.dot(PVector.sub(support,vertices[i]));
    
    //leastVertex stores the greatest penetration distance
    if(leastVertex.val < penetrationDistance) {
      leastVertex.val = penetrationDistance;
      leastVertex.idx = i;
    }
  }
  return leastVertex;
}

ArrayList<PVector> findIncidentFace(Polygon reference, Polygon incident, int refIdx){
  ArrayList<PVector> incidentFace = new ArrayList<PVector>();
  
  Vertex face = new Vertex();
  face.val = Float.MAX_VALUE;
  
  PVector[] refVertices = reference.convertLocalToWorld();
  int refIdx2 = (refIdx+1) % (refVertices.length);
  PVector normal = getNormalTo(refVertices[refIdx], refVertices[refIdx2]);
  
  PVector[] incVertices = incident.convertLocalToWorld();
  int numIncVertices = incVertices.length;
  
  //iteratively find the incidentface
  for( int i = 0; i < numIncVertices; i++) {
    int j = (i+1) % numIncVertices;
    PVector incidentNormal = getNormalTo(incVertices[i], incVertices[j]);
    float dot = PVector.dot(normal, incidentNormal);
    
    if(dot < face.val) {
      face.val = dot;
      face.idx = i;
    }
  }
  
  //add vertices of incident face and return
  incidentFace.add(incVertices[face.idx]);
  incidentFace.add(incVertices[(face.idx+1) % numIncVertices]);
  
  return incidentFace;
}

//credit: this one is nearly identical translation from randy gaul's impulse engine
int clip(PVector n, float c, ArrayList<PVector> face) {
  int sp = 0;
  PVector [] out = {
    face.get(0),
    face.get(1)
  };

  // Retrieve distances from each endpoint to the line
  // d = ax + by - c
  float d1 = n.dot( face.get(0) ) - c;
  float d2 = n.dot( face.get(1) ) - c;

  // If negative (behind plane) clip
  if(d1 <= 0.0f) {
    out[sp++] = face.get(0);
  }
  if(d2 <= 0.0f) {
    out[sp++] = face.get(1);
  }
  
  // If the points are on different sides of the plane
  if(d1 * d2 < 0.0f) {// less than to ignore -0.0f
    // Push interesection point
    float alpha = d1 / (d1 - d2);
    out[sp] = PVector.lerp( face.get(0), face.get(1), alpha );
    sp++;
  }

  // Assign our new converted values
  face.set(0,out[0]);
  face.set(1,out[1]);
  
  assert( sp != 3 );
  
  return sp;
}

//collision cases//-------------------------------------------

void CircleCircle(Circle a, Circle b, Manifold m){
  PVector distanceAB = PVector.sub(b.position, a.position);
  float magSq = distanceAB.magSq(); //magnitude squared
  float tRadius = a.radius + b.radius;
  float distance = sqrt(magSq);
  
  //no collision
  if(magSq > sq(tRadius)) return;
  
  //add contact point to manifold
  if(distance == 0.0f) {
    m.penetration = a.radius;
    m.normal = new PVector(1,0,0);
    m.contacts.add(a.position);
  } else {
    m.penetration = tRadius - distance;
    m.normal = PVector.mult(distanceAB, (1.0/distance) );
    PVector contactPoint = PVector.add(a.position,PVector.mult(m.normal,a.radius));
    m.contacts.add(contactPoint);
  }
}

void CirclePolygon(Circle a, Polygon b, Manifold m){
  float distSq = PVector.sub(b.position,a.position).magSq();
  
  //no collision
  if (distSq > sq(a.radius + b.maxRadius) ) return;
  
  PVector center = a.position; //circle center
  PVector[] vertices = b.convertLocalToWorld();
  int numVertices = vertices.length;
  
  //find edge with minimum penetration
  Vertex closest = new Vertex();
  for(int i=0; i < numVertices; i++) {
    int j = (i+1) % numVertices;
    PVector normal = getNormalTo(vertices[i],vertices[j]);
    float newSeparation = normal.dot(PVector.sub(center,vertices[i]));
    
    //not the correct edge
    if(newSeparation > a.radius) return;
    
    if(newSeparation > closest.val) {
      closest.val = newSeparation;
      closest.idx = i;
    }
  }
  
  //now that the face is found, use the two vertices forming it
  PVector vertexA = vertices[closest.idx];
  PVector vertexB = vertices[(closest.idx+1) % numVertices];
  
  //check if the center is inside of the polygon
  if(closest.val < 1e-6) {
    m.normal = PVector.mult(getNormalTo(vertexA,vertexB), -1);
    PVector contactPoint = PVector.add(a.position,PVector.mult(m.normal,a.radius));
    m.contacts.add(contactPoint);
    m.penetration = a.radius;
    return;
  }
  
  //finding the voronoi region
  float dot1 = PVector.dot( PVector.sub(center,vertexA), PVector.sub(vertexB,vertexA) );
  float dot2 = PVector.dot( PVector.sub(center,vertexB), PVector.sub(vertexA,vertexB) );
  m.penetration = a.radius - closest.val;
  
  //three cases, closest to vertexA (dot1 check), vertexB (dot2 check), or to the face
  if(dot1 <= 0) {
    if ( PVector.sub(center,vertexA).magSq() > sq(a.radius)) return;
    m.normal = PVector.sub(vertexA, center).normalize();
    m.contacts.add(vertexA);
  } else if(dot2 <= 0) {
    if(PVector.sub(center, vertexB).magSq() > sq(a.radius)) return;
    m.normal = PVector.sub(vertexB, center).normalize();
    m.contacts.add(vertexB);
  } else {
    PVector normal2 = getNormalTo(vertexA, vertexB);
    if( PVector.dot( PVector.sub(center, vertexA), normal2) > a.radius ) return;
    m.normal = PVector.mult(normal2,-1);
    PVector contactPoint = PVector.add(a.position, PVector.mult(m.normal, a.radius));
    m.contacts.add(contactPoint);
  }
}

void PolygonPolygon(Polygon a, Polygon b, Manifold m){
  
  Vertex penA = findAxisOfLeastPenetration(a,b);
  Vertex penB = findAxisOfLeastPenetration(b,a);
  
  //separating axis found, no collision
  if(penA.val >= 0 || penB.val >= 0) return;
  
  int faceIdx;
  boolean flip;
  
  Polygon reference;
  Polygon incident;
  
  //finding the shape which has the reference face
  if(penA.val > penB.val) {
    reference = a; incident = b;
    faceIdx = penA.idx;
    flip = false;
  } else {
    reference = b; incident = a;
    faceIdx = penB.idx;
    flip = true;
  }
  
  ArrayList<PVector> face = findIncidentFace(reference, incident, faceIdx);
  
  //reference face vertices, and converting to world frame
  PVector[] referenceVertices = reference.convertLocalToWorld();
  PVector vertexOne = referenceVertices[faceIdx];
  PVector vertexTwo = referenceVertices[(faceIdx+1) % referenceVertices.length];
  
  //reference face side normal (world frame)
  PVector sidePlaneNormal = PVector.sub(vertexTwo, vertexOne);
  sidePlaneNormal.normalize();
  
  //orthogonalize for reference face normal
  PVector refFaceNormal = new PVector(sidePlaneNormal.y, -sidePlaneNormal.x);
  
  //distance from ref to origin
  float refDistOrig = PVector.dot(vertexOne, refFaceNormal);
  float negSide = -PVector.dot(vertexOne, sidePlaneNormal);
  float posSide = PVector.dot(vertexTwo, sidePlaneNormal);
  
  //clipping incident face to reference face side planes
  if(clip( PVector.mult(sidePlaneNormal,-1), negSide, face) < 2) 
    return; //both returns are incase we don't have required points due to floating point error 
  if(clip( sidePlaneNormal, posSide, face) < 2) 
    return;
  
  //flip based on which shape has reference face
  m.normal = PVector.mult(refFaceNormal, flip ? -1 : 1);
  
  int clippedPoints = 0; //clipped points behind reference face
  float separation = refFaceNormal.dot(face.get(0)) - refDistOrig;
  
  if(separation <= 0) {
    m.contacts.add(face.get(0));
    m.penetration = -separation;
    clippedPoints++;
  } else {
    m.penetration = 0;
  }
  
  separation = refFaceNormal.dot(face.get(1)) - refDistOrig;
  
  if(separation <= 0) {
    m.contacts.add(face.get(1));
    m.penetration += -separation;
    clippedPoints++;
    m.penetration /= clippedPoints; //making the manifold pen the average penetration
  }  
}

void collisionHandler(){
  int size = bodyList.size();
  for(int i = 0; i < size; i++) {
    Body b1 = bodyList.get(i);
    for(int j = i+1; j < size; j++) {
      Body b2 = bodyList.get(j);
      
      //check for infinite mass
      if(b1.getInverseMass()==0 && b2.getInverseMass()==0) continue;
      
      //create manifold
      Manifold m = new Manifold(b1,b2);
      m.triggerCollisionHandler();
      
      //if the manifold
      if(m.contacts.size()>0) {
        manifolds.add(m);
      }
    }
  }
}
