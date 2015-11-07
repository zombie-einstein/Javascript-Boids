'use strict';
//****************** Boid Class *********************

// Boid Constructor
function boid(){
	this.position = new vec(0,0);
	this.velocity = new vec(0,0);
	this.accVec = new vec(0,0);
	this.predictionVec = new vec(0,0);
	this.predictedPosition = new vec(0,0);
	this.neighbourAvg = new vec(0,0);
	this.neighbourVel = new vec(0,0);
	this.collisionsAvg = new vec(0,0);
	this.numNeighbours = 0;
	this.numCollisions = 0;
	this.triangle = [new vec(0,0), new vec(0,0), new vec(0,0)];
	this.leftOrRight = Math.round(Math.random())*2 -1; // Randomizes the boids tendency to turn left or right first 
}
// Avoid the walls
boid.prototype.wallAvoid = function () {
	this.predictionVec.assign(this.velocity);
	this.predictionVec.setMagnitude(detectionRange);
	var r = 0;
	this.predictedPosition.assign(this.position);
	this.predictedPosition.add(this.predictionVec);
	// This check is for the borders, but could be generalised to other
	// shapes as long as this check can be performed
	while ( this.predictedPosition.x > xWidth || this.predictedPosition.x < 0 || this.predictedPosition.y > yWidth || this.predictedPosition.y < 0 ){
		this.predictedPosition.subtract(this.predictionVec);
		this.predictionVec.rotate(this.leftOrRight*Math.pow(-1,r) * (r+1) * 2*Math.PI/8);
		this.predictedPosition.add(this.predictionVec);		
		r += 1;
		if (r > 14 ){break;}	// Breaks out of loop if the boid can't find an exit!
	}	
}
// Give boid random initial position and velocity
boid.prototype.initial = function () {
	var randomVecA = new vec( Math.random()*(xWidth-1) +1 , Math.random()*(yWidth-1) +1 );
	this.position.add(randomVecA);
	var randomVecB = new vec( Math.random()*2*maxVelocity - maxVelocity , Math.random()*2*maxVelocity- maxVelocity );
	this.velocity.add(randomVecB);		
}
// Update Boid position function ( add velocity to position )
boid.prototype.move = function () {
	this.velocity.add(this.accVec);
	this.velocity.maxLimit(maxVelocity);
	this.position.add(this.velocity);
}
// Work out triangle points
boid.prototype.triVec = function() {
	this.triangle[0].assign(this.velocity);
	this.triangle[0].setMagnitude(5+3);
	this.triangle[1].assign(this.triangle[0]);
	this.triangle[1].setMagnitude(5);
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
	ctx.fillStyle = '#FFFFFF'
	ctx.fill();					
}
// Reset the averages and counts
boid.prototype.reset = function() {
	this.accVec.reset();
	this.neighbourAvg.reset();
	this.neighbourVel.reset();
	this.collisionsAvg.reset();
	this.numNeighbours = 0;
	this.numCollisions = 0;
}
// Add 'neighbour' position and velocity to average
boid.prototype.neighbourAvgs = function(neighbour){ 
	this.numNeighbours += 1;	// increment number of neighbours
	this.neighbourAvg.cumAvg(neighbour.position,this.numNeighbours);
	this.neighbourVel.cumAvg(neighbour.velocity,this.numNeighbours);
}
// Add 'collisions' to average
boid.prototype.collisionAvgs = function(neighbour,d){ 
	this.numCollisions += 1;	// increment number of neighbours
	var tempVec = new vec(0,0);
	tempVec.assign(this.position);
	tempVec.subtract(neighbour.position);
	tempVec.scale(1/d);
	this.collisionsAvg.cumAvg(tempVec,this.numCollisions);
}
// Calculate new velocity based on neighbours
boid.prototype.acceleration = function(){
	if ( this.numNeighbours > 0 ) {
		// Cohesion Calculation
		var vecToAvg = new vec(0,0);
 		vecToAvg.assign(this.neighbourAvg);
		vecToAvg.subtract(this.position);
		vecToAvg.setMagnitude(maxVelocity);
		var steerCohesion = new vec(0,0);
		steerCohesion.assign(vecToAvg);
		steerCohesion.subtract(this.velocity);
		steerCohesion.maxLimit(maxSteering);
		// Alignment Calculation
		this.neighbourVel.setMagnitude(maxVelocity);
		var steerAlign = new vec(0,0);
		steerAlign.assign(this.neighbourVel);
		steerAlign.subtract(this.velocity);
		steerAlign.maxLimit(maxSteering);
		// Avoid calculation
		var steerAvoid = new vec(0,0);
		if ( this.numCollisions > 0 ){
			this.collisionsAvg.setMagnitude(maxVelocity);
			steerAvoid.assign(this.collisionsAvg);
			steerAvoid.subtract(this.velocity);
			steerAvoid.maxLimit(maxSteering);
		}
		// Scale steering contributions by weight
		steerCohesion.scale(1);
		steerAlign.scale(1.5);
		steerAvoid.scale(2);
		// Calculate acceleration
		this.accVec.add(steerCohesion);
		this.accVec.add(steerAlign);
		this.accVec.add(steerAvoid);
	}
	// Avoid the walls
	this.wallAvoid();
	this.predictionVec.setMagnitude(maxVelocity);
	this.predictionVec.subtract(this.velocity);
	this.predictionVec.maxLimit(maxSteering);
	this.predictionVec.scale(8);
	this.accVec.add(this.predictionVec);
}