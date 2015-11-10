'use strict';
//****************** Boid Class *********************

// Boid Constructor
function boid(x,y,vx,vy){
	this.position = new vec(x,y);
	this.velocity = new vec(vx,vy);
	this.accVec = new vec(0,0);
	this.predictionVec = new vec(0,0);
	this.predictedPosition = new vec(0,0);
	this.steerCohesion = new vec(0,0);
	this.steerAlign = new vec(0,0);
	this.steerAvoid = new vec(0,0);
	this.numNeighbours = 0;
	this.numCollisions = 0;
	this.triangle = [new vec(0,0), new vec(0,0), new vec(0,0)];
	// Randomizes the boids tendency to turn left or right first
	// when encountering a boundary / barrier
	this.leftOrRight = Math.round(Math.random())*2 -1;
	this.color = shadeColor2("#FF9900", Math.random()*0.4-0.2);
}

// Avoid the walls and circular obstacles
boid.prototype.wallAvoid = function () {
	this.predictionVec.assign(this.velocity);
	this.predictionVec.setMagnitude(detectionRange);
	var r = 0;
	this.predictedPosition.assign(this.position);
	this.predictedPosition.add(this.predictionVec);
	// This check is for the borders and also for circular obstacles
	while ( this.predictedPosition.boundaryDetect() == true || this.predictedPosition.obstacleDetect(7.5) == true ) {
		this.predictedPosition.subtract(this.predictionVec);
		this.predictionVec.rotate(this.leftOrRight*Math.pow(-1,r) * (r+1) * 2*Math.PI/detectAngle);
		this.predictedPosition.add(this.predictionVec);		
		r += 1;
		if (r > detectAngle-2 ){break;}	// Breaks out of loop if the boid can't find an exit (has checked all 2PI)!
	}	
}

// Give boid random initial position and velocity
boid.prototype.randomize = function () {
	this.position = randomPosition();
	this.velocity = randomVelocity();		
}

// Update Boid position function 
//( i.e. add acceleration to the velocity, and velocity to position )
boid.prototype.move = function () {
	this.velocity.add(this.accVec);
	this.velocity.maxLimit(maxVelocity);
	this.position.add(this.velocity);
}
// Work out triangle render points by scaling and rotating velocity vector
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
	ctx.fillStyle =  this.color;
	ctx.fill();					
}

// Reset the neighbour averages and counts
boid.prototype.reset = function() {
	this.accVec.reset();
	this.steerCohesion.reset();
	this.steerAlign.reset();
	this.steerAvoid.reset();
	this.numNeighbours = 0;
	this.numCollisions = 0;
}

// Add 'neighbour' position and velocity to average
boid.prototype.neighbourAvgs = function(neighbour){ 
	this.numNeighbours += 1;	// increment number of neighbours
	this.steerCohesion.cumAvg(neighbour.position,this.numNeighbours);
	this.steerAlign.cumAvg(neighbour.velocity,this.numNeighbours);
}

// Adds position of neighbours that are too close to average
boid.prototype.collisionAvgs = function(neighbour,d){ 
	this.numCollisions += 1;	// increment number of neighbours
	var tempVec = new vec(0,0);
	tempVec.assign(this.position);
	tempVec.subtract(neighbour.position);
	// Contributions are weighted by their distance
	// i.e. the closer the boid the stronger it's contribution
	tempVec.scale(1/d);
	this.steerAvoid.cumAvg(tempVec,this.numCollisions);
}

// Calculate new velocity based on neighbours
boid.prototype.acceleration = function(){
	if ( this.numNeighbours > 0 ) {
		// Cohesion Calculation
		this.steerCohesion.subtract(this.position);
		this.steerCohesion.setMagnitude(maxVelocity);
		this.steerCohesion.subtract(this.velocity);
		this.steerCohesion.maxLimit(maxSteering);
		// Alignment Calculation
		this.steerAlign.subtract(this.velocity); // Does this need scaling? It is already an average and so can't exceed max velocity anyway?
		this.steerAlign.maxLimit(maxSteering);
		// Avoid calculation
		if ( this.numCollisions > 0 ){
			this.steerAvoid.setMagnitude(maxVelocity);
			this.steerAvoid.subtract(this.velocity);
			this.steerAvoid.maxLimit(maxSteering);
		}
		// Scale steering contributions by arbitary weights
		this.steerCohesion.scale(cohesionStrength);
		this.steerAlign.scale(alignStrength);
		this.steerAvoid.scale(avoidStrength);
		// Calculate overall acceleration due to neighbours
		this.accVec.add(this.steerCohesion);
		this.accVec.add(this.steerAlign);
		this.accVec.add(this.steerAvoid);
	}
	// Avoid the walls and objects and add effect to acceleration
	this.wallAvoid();
	this.predictionVec.setMagnitude(maxVelocity);
	this.predictionVec.subtract(this.velocity);
	this.predictionVec.maxLimit(maxSteering);
	this.predictionVec.scale(10);
	this.accVec.add(this.predictionVec);
}