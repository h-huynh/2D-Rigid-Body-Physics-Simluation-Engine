import java.util.*;

class Manifold {
  Body a, b;
  ArrayList<PVector> contacts = new ArrayList<PVector>();
  float penetration = 0; //total depth of penetration which the two colliding shapes have caused
  float tStaticFriction = 0; //total static friction caused by mix of a & b
  float tDynamicFriction = 0; //total dynamic friction caused by mix of a & b
  float e = 0; //coefficient of restitution, taken from mix of a and b
  PVector zeroV = new PVector(0,0,0);
  PVector normal = new PVector();
  
  //constructor
  Manifold(Body a,Body b) {
    this.a=a;
    this.b=b;
    
    //note to self: although this is how it is done in gaul's impulse engine,
    //is this a simplication of how restitution works?
    e = min(a.restitution,b.restitution);
    
    tStaticFriction = sqrt( a.staticFriction * b.staticFriction );
    tDynamicFriction = sqrt( a.dynamicFriction * b.dynamicFriction );
  }
  
  void applyImpulse() {
    float numContacts = contacts.size();

    //zero contact case
    if(numContacts==0) return;
    
    //infinite mass case (0 mass was defined as infinite)
    if(a.getInverseMass()==0 && b.getInverseMass()==0) {
      a.velocity = zeroV;
      b.velocity = zeroV;
      return;
    }
    
    for( PVector contactPoint : contacts ) {
      PVector velA = a.netVelocityAtPosition(contactPoint);
      PVector velB = b.netVelocityAtPosition(contactPoint);
      
      //relative velocity
      PVector velR = PVector.sub(velB,velA);
      
      //contact velocity is the dot of the normal and velAB
      float contactVel = PVector.dot(velR,normal);
      
      if(contactVel>0) continue; //treat contacts of very low velocities as 0
    
      PVector distA = a.getDistance(contactPoint);
      PVector distB = b.getDistance(contactPoint);
      float normDistA = distA.cross(normal).z;
      float normDistB = distB.cross(normal).z;
      float inverseMassSum = a.getInverseMass() + b.getInverseMass() 
                           + sq(normDistA) * a.getInverseMomentOfInertia() 
                           + sq(normDistB) * b.getInverseMomentOfInertia();
      
      //calculate restitution  
      float restitutionImpulse = -(1.0f + e) * contactVel;
      restitutionImpulse /= (inverseMassSum * numContacts);
     
      //apply restitution impulses
      a.contactImpulse(distA, PVector.mult(normal,-restitutionImpulse));
      b.contactImpulse(distB, PVector.mult(normal, restitutionImpulse));

      PVector tangentImpulse = PVector.sub(velR,PVector.mult(normal,PVector.dot(velR,normal)));
      
      //friction is treated as a tangent impulse
      tangentImpulse.normalize();
      float frictionMag = -velR.dot(tangentImpulse);
      frictionMag /= (inverseMassSum * numContacts);
      float fric = abs(frictionMag);
      
      //ignore small friction impulses
      if(fric < 1e-4) continue;
      
      //check if static friction impulse needs to be applied, else dynamic
      if(fric < restitutionImpulse * tStaticFriction) {
        tangentImpulse.mult(frictionMag);
      } else {
        tangentImpulse.mult(-restitutionImpulse * tDynamicFriction);
      }
      
      //apply friction impulses
      a.contactImpulse(distA, PVector.mult(tangentImpulse,-1));
      b.contactImpulse(distB, tangentImpulse);
    }
  }
  
  //send collision info to CollisionHandler functions
  //there are 4 cases for (a b):
  //(circle circle) (circle polygon) (polygon circle) (polygon polygon)
  void triggerCollisionHandler(){
    //(circle ___)
    if (a instanceof Circle){
      if (b instanceof Circle) {
        CircleCircle((Circle) a, (Circle) b, this);
      } else if (b instanceof Polygon){
        CirclePolygon((Circle) a, (Polygon) b, this);
      }
    }
    //(polygon ___)
    else if (a instanceof Polygon) {
      if (b instanceof Polygon){
        PolygonPolygon( (Polygon) a, (Polygon) b,this);
      } else if (b instanceof Circle){
        //need to swap a and b for CirclePolygon()
        Body tempBody = a; a = b; b = tempBody;
        CirclePolygon((Circle) a, (Polygon) b,this);
      }
    }
  }
  
  //fixes the positional penetration of a and b
  void fixPenetration(){
    float k_slop = 0.04f; //how far the penetration is allowed before being fixed
    float percent = 0.4f; //percent penetration to fix
    
    PVector correction = PVector.mult(this.normal,(max( penetration - k_slop, 0.0f ) 
    / (a.getInverseMass() + b.getInverseMass())) * percent);
    
    a.position.sub(PVector.mult(correction, a.getInverseMass()));
    b.position.add(PVector.mult(correction, b.getInverseMass()));
  }
  
}//manifold
