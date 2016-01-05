'use strict';
//****************** Boid Class & associated functions ********************

// Boid Constructor
function boid(){
	this.position 		= new vec(0,0);	// Boids position
	this.velocity 		= new vec(0,0);	// Boids velocity
	this.accVec 		= new vec(0,0); // Boids acceleration vector, newly calculated at each timestep
	this.steerCohesion 	= new vec(0,0); // Steering vector to average position of neighbours
	this.steerAlign 	= new vec(0,0);	// Steering vector to align boid velocity with neighbours
	this.steerAvoid 	= new vec(0,0);	// Steering vector to avoid other boids
	this.steerObstacle 	= new vec(0,0);	// Steering vector to avoid walls and obstacles
	this.numNeighbours 	= 0;			// Count the boids neighbours
	this.numCollisions 	= 0;			// Count the number of boids that are in collision range
	this.numObstacles	= 0;			// Count the number of walls or obstacles to be avoided
	this.alive 			= true;			// Status of boid
	this.triangle 		= [new vec(0,0), new vec(0,0), new vec(0,0)];		// Points of triangle used to render boid
	this.color 			= shadeColor2( "#FF9900", Math.random()*0.3 -0.15 );	// Give each boid a random shade from a base colour
}

// Avoid the circular obstacles
boid.prototype.obstacleAvoid = function( offset, range ){
	for ( var n = 0; n < numObstacles; n++ ){
		var d = this.position.squareDistance( Obstacles[n].centre );
		// First check if boid is in range of the obstacle
		if ( d < square( range +offset +Obstacles[n].radius ) ){
			var dToTangent 		= Math.sqrt( d +square( Obstacles[n].radius ) );	// Distance to the tangents of the obstacle
			var vecToObstacle 	= new vec(Obstacles[n].centre.x -this.position.x, Obstacles[n].centre.y -this.position.y); //Vector from Boid to obstacle
			var cosToTangent 	= Obstacles[n].radius /dToTangent;				// Cos of angle from centre of obstacle to tangent
			var cosToVelocity 	= this.velocity.cosAngle( vecToObstacle );		// Cos of angle between velocity of boid and vector to obstacle
			// Secondly check if Boid will eventually intercept obstacle
			if ( cosToVelocity > cosToTangent ){
				vecToObstacle.setMagnitude( Math.sqrt(d) -offset -Obstacles[n].radius );
				// Generate temporary boid at obstacle boundary closest to boid
				var boundVec 	= new vec( this.position.x +vecToObstacle.x, this.position.y +vecToObstacle.y );
				// Add vector pointing away from obstacle boid to average vector
				this.numObstacles ++;
				this.steerObstacle.collisionAvgs( this, boundVec, range, this.numObstacles );
			}
		}
	}
}

// Avoid walls
boid.prototype.wallAvoid = function( offset, range ){
	if ( this.position.x < offset +range && this.velocity.x < 0 ){
			var boundVec = new vec( offset, this.position.y );
			this.numObstacles ++;
			this.steerObstacle.collisionAvgs( this, boundVec, range, this.numObstacles );
		}
	if ( this.position.x > xWidth -offset -range && this.velocity.x > 0 ){
			var boundVec = new vec( xWidth -offset, this.position.y );
			this.numObstacles ++;
			this.steerObstacle.collisionAvgs( this, boundVec, range, this.numObstacles );
		}
	if ( this.position.y < offset +range && this.velocity.y < 0 ){
			var boundVec = new vec( this.position.x, this.numObstacles );
			this.numObstacles ++;
			this.steerObstacle.collisionAvgs( this, boundVec, range, this.numObstacles );
		}
	if ( this.position.y > yWidth -offset -range && this.velocity.y > 0 ){
			var boundVec = new vec( this.position.x, yWidth -offset );
			this.numObstacles ++;
			this.steerObstacle.collisionAvgs( this, boundVec, range, this.numObstacles );
		}
}

// Adds vector that points away from vector to average
vec.prototype.collisionAvgs = function( object, subject, range, count ){
	var tempVec = new vec( object.position.x -subject.x, object.position.y -subject.y );
	tempVec.setMagnitude( range -tempVec.magnitude() );
	this.cumAvg( tempVec, count );
}

// Add 'neighbour' position and velocity to average
boid.prototype.neighbourAvgs = function( neighbour ){ 
	this.numNeighbours += 1;	// increment number of neighbours
	this.steerCohesion.cumAvg(neighbour.position,this.numNeighbours);
	this.steerAlign.cumAvg(neighbour.velocity,this.numNeighbours);
}

// Give boid random initial position and velocity
boid.prototype.randomize = function () {
	this.position = randomPosition();
	this.velocity = randomVelocity( maxVelocity );		
}

// Update Boid position function 
//( i.e. add acceleration to the velocity, and velocity to position )
boid.prototype.move = function () {
	//this.velocity.add(this.accVec);
	this.velocity.maxLimit(maxVelocity);	// Double check that the velocity does not exceed the maximum velocity after acceleration
	this.position.add(this.velocity);		
	// Hard wall against obstacles, stops boids if they try to enter an obstacle
	//if (this.position.obstacleDetect(5)){
	//	this.position.subtract(this.velocity); 
	//}
}
// Work out triangular boid render points by scaling and rotating velocity vector
boid.prototype.triVec = function() {
	this.triangle[0].assign(this.velocity);
	this.triangle[0].setMagnitude(boidSize*1.5);
	this.triangle[1].assign(this.triangle[0]);
	this.triangle[1].setMagnitude(boidSize);
	this.triangle[1].rotate(2*Math.PI/3);
	this.triangle[2].assign(this.triangle[1]);
	this.triangle[2].rotate(2*Math.PI/3);
}

// Render the boid on screen
boid.prototype.render = function() {	
	this.triVec();
	ctx.beginPath();
	ctx.moveTo(this.position.x+this.triangle[0].x,this.position.y+this.triangle[0].y);
	ctx.lineTo(this.position.x+this.triangle[1].x,this.position.y+this.triangle[1].y);
	ctx.lineTo(this.position.x+this.triangle[2].x,this.position.y+this.triangle[2].y);
	ctx.fillStyle =  this.color;
	ctx.fill();				
}

// Draw steering vectors and vision range
boid.prototype.drawSteering = function(){

	ctx.beginPath();
	ctx.arc( this.position.x, this.position.y, detectionRange, 0, 2*Math.PI );
	ctx.strokeStyle = "rgba(255, 0, 0, 0.25)";
	ctx.stroke();

	ctx.beginPath();
	ctx.arc( this.position.x, this.position.y, collisionRange, 0, 2*Math.PI );
	ctx.fillStyle = "rgba(255, 0, 0, 0.25)";
	ctx.fill();

	if (this.numNeighbours > 0){
		ctx.beginPath();
		ctx.moveTo(this.position.x,this.position.y);
		ctx.lineTo(this.position.x+this.steerAlign.x,this.position.y+this.steerAlign.y)
		ctx.strokeStyle = '#ff0000';	// Red
		ctx.stroke();

		ctx.beginPath();
		ctx.moveTo(this.position.x,this.position.y);
		ctx.lineTo(this.steerCohesion.x,this.steerCohesion.y)
		ctx.strokeStyle = '#0000ff';	// Blue
		ctx.stroke();
	}
	if (this.numCollisions > 0){
		ctx.beginPath();
		ctx.moveTo(this.position.x,this.position.y);
		ctx.lineTo(this.position.x+this.steerAvoid.x,this.position.y+this.steerAvoid.y)
		ctx.strokeStyle = '#00ff00';	// Green
		ctx.stroke();
	}
	if (this.numObstacles > 0){
		ctx.beginPath();
		ctx.moveTo(this.position.x,this.position.y);
		ctx.lineTo(this.position.x+this.steerObstacle.x,this.position.y+this.steerObstacle.y)
		ctx.strokeStyle = '#00ff00';	// Green
		ctx.stroke();
	}
}

// Reset the neighbour averages and counts
boid.prototype.reset = function() {
	this.accVec.reset();
	this.steerCohesion.reset();
	this.steerAlign.reset();
	this.steerAvoid.reset();
	this.steerObstacle.reset();
	this.numNeighbours	= 0;
	this.numCollisions	= 0;
	this.numObstacles	= 0;
}

// Calculate new velocity based on neighbours
boid.prototype.acceleration = function(){
	
	if ( this.numNeighbours > 0 ) {
		// Cohesion Calculation
		this.steerCohesion.subtract(this.position);	// Vector now points to average neighbour position
		this.steerCohesion.maxLimit(maxVelocity);	// Limit the size of this vector
		//this.steerCohesion.subtract(this.velocity);	// Steer velocity towards cohesion vector
		this.steerCohesion.maxLimit(maxSteering);	// Limit by steering strength
		// Alignment Calculation
		//this.steerAlign.subtract(this.velocity);	// Steering vector towards neighbour average velocity
		//this.steerAlign.maxLimit(maxSteering);		// Limit to maximium steering
		// Scale steering contributions by arbitary weights
		this.steerCohesion.scale(cohesionStrength);
		this.steerAlign.scale(alignStrength);
	}
	
	// Avoid neighbour calculation
	if ( this.numCollisions > 0 ){
		//this.steerAvoid.subtract(this.velocity);	// Steering vector from velocity
		this.steerAvoid.maxLimit(maxVelocity);		// Limit avoid vector to max velocity
		//this.steerAvoid.maxLimit(maxSteering);		// Limit steering vector to max steering
		// Scale steering contributions by arbitary weights
		this.steerAvoid.scale(avoidStrength);
	}
	
	// Avoid obstacles calculation
	if ( this.numObstacles > 0 ){
		//this.steerObstacle.subtract(this.velocity);	// Steering vector from velocity
		this.steerObstacle.maxLimit(maxVelocity);	// Limit steering vector to max steering
		// Scale steering contributions by arbitary weights
		this.steerObstacle.scale(obstacleStrength);
	}
	
	// Speed up vector steering
	//var speedUp = new vec(this.velocity.x,this.velocity.y);
	//speedUp.setMagnitude(maxVelocity);
	//speedUp.subtract(this.velocity);
	//speedUp.maxLimit(maxSteering);
	//speedUp.scale(speedUpStrength);
	

	// Calculate overall acceleration due to neighbours
	//this.accVec.add(speedUp);
	//this.accVec.add(this.steerCohesion);
	//this.accVec.add(this.steerAlign);
	//this.accVec.add(this.steerAvoid);
	//this.accVec.add(this.steerObstacle);

	if ( this.numNeighbours > 0 || this.numCollisions > 0 || this.numObstacles > 0 ){
		var desiredVec = new vec(0,0);
		desiredVec.assign( this.steerCohesion );
		desiredVec.add( this.steerAlign );
		desiredVec.add( this.steerAvoid );
		desiredVec.add( this.steerObstacle );

		if ( desiredVec.cosAngle( this.velocity ) > cosMaxSteerAngle ){
			this.velocity.assign( desiredVec );
			this.velocity.setMagnitude( maxVelocity );
		}
		else{
			if ( this.velocity.crossProduct( desiredVec ) > 0 ){
				//document.getElementById("testValues").innerHTML = this.velocity.crossProduct( desiredVec );
				this.velocity.rotate( maxSteerAngle );
			}
			else{
				this.velocity.rotate( -maxSteerAngle );
			}

		}
	}
}

// Return vector to another boid
boid.prototype.vecTo = function( subject ){
	return new vec(subject.position.x -this.position.x, subject.position.y -this.position.y);
}

// Neighbour check function
function neighbourTest(){
	// The arrangement of these loops means that neighbour checks are only
	// made once between pairs of boids, slightly more effecient than
	// check all boids for each boid	
	for ( var n = 0; n < numBoids-1; n++ ){
	for ( var m = n+1; m < numBoids; m++ ){ 
		// This checks that neighbours are both within a certain range
		var d = Boids[n].position.squareDistance(Boids[m].position);
		if ( d < square(detectionRange) ){
				// Check if neighbour is inside vision cone
				if ( Boids[n].velocity.cosAngle(Boids[n].vecTo(Boids[m])) >  detectionAngle){
					Boids[n].neighbourAvgs(Boids[m]);
					if ( d < square(collisionRange) ){
						Boids[n].numCollisions ++;
						Boids[n].steerAvoid.collisionAvgs( Boids[n], Boids[m].position, collisionRange, Boids[n].numCollisions ); 
					}
				}
				// Check if neighbour is inside vision cone
				if ( Boids[m].velocity.cosAngle(Boids[m].vecTo(Boids[n])) >  detectionAngle){
					Boids[m].neighbourAvgs(Boids[n]);
					if ( d < square(collisionRange) ){
						Boids[m].numCollisions ++;
						Boids[m].steerAvoid.collisionAvgs( Boids[m], Boids[n].position, collisionRange, Boids[m].numCollisions ); 
					}
				}
	}} //subject, range, avoidVector, count
}
	for ( var n = 0; n < numBoids; n++ ){
		Boids[n].obstacleAvoid( 5, detectionRange );
		Boids[n].wallAvoid( 5, detectionRange );
	}

}
