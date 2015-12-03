'use strict';
//****************** Boid Class *********************

// Boid Constructor
function boid(x,y,vx,vy){
	this.position 	= new vec(x,y);
	this.velocity 	= new vec(vx,vy);
	this.accVec 	= new vec(0,0);
	this.predictionVecA = new vec(0,0);
	this.predictionVecB = new vec(0,0);
	this.predictedPositionA = new vec(0,0);
	this.predictedPositionB = new vec(0,0);
	this.steerCohesion 	= new vec(0,0);
	this.steerAlign = new vec(0,0);
	this.steerAvoid = new vec(0,0);
	this.numNeighbours = 0;
	this.numCollisions = 0;
	this.alive = true;
	this.triangle = [new vec(0,0), new vec(0,0), new vec(0,0)];
	// Randomizes the boids tendency to turn left or right first
	// when encountering a boundary / barrier
	this.leftOrRight = Math.round(Math.random())*2 -1;
	// Give boid a random shade
	this.color = shadeColor2("#FF9900", Math.random()*0.3-0.15);
}

// Avoid the walls and circular obstacles
boid.prototype.wallAvoid = function () {
	this.predictionVecA.assign(this.velocity);
	this.predictionVecB.assign(this.velocity);
	this.predictionVecA.setMagnitude(detectionRange);
	this.predictedPositionA.assign(this.position);
	this.predictedPositionA.add(this.predictionVecA);
	this.predictedPositionB.assign(this.position);
	this.predictedPositionB.add(this.predictionVecB);
	var r = 0;
	// This check is for the borders and also for circular obstacles
	while ( this.predictedPositionA.boundaryDetect() == true || this.predictedPositionA.obstacleDetect(5) == true ||  this.predictedPositionB.obstacleDetect(5) == true) {
	//while ( this.predictedPosition.boundaryDetect() == true || this.circleTestB(detectionRange) == true ) {
		this.predictedPositionA.subtract(this.predictionVecA);
		this.predictionVecA.rotate(this.leftOrRight*Math.pow(-1,r) * (r+1) * 2*Math.PI/detectAngle);
		this.predictedPositionA.add(this.predictionVecA);
		this.predictedPositionB.subtract(this.predictionVecB);
		this.predictionVecB.rotate(this.leftOrRight*Math.pow(-1,r) * (r+1) * 2*Math.PI/detectAngle);
		this.predictedPositionB.add(this.predictionVecB);	
		r += 1;
		if (r > detectAngle-1 ){
		this.alive = false; //
		this.color = '#ff0000';
		break;}	// Breaks out of loop if the boid can't find an exit (has checked all 2PI)!
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
	// Hard wall against obstacles, stops boids if they try to enter an obstacle
	if (this.position.obstacleDetect(5)){
		this.position.subtract(this.velocity); 
	}
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
	tempVec = neighbour.vecTo(this);
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
		this.steerCohesion.maxLimit(maxVelocity);
		this.steerCohesion.subtract(this.velocity);
		this.steerCohesion.maxLimit(maxSteering);
		// Alignment Calculation
		this.steerAlign.subtract(this.velocity); // Does this need scaling? It is already an average and so can't exceed max velocity anyway?
		this.steerAlign.maxLimit(maxSteering);
		// Avoid calculation
		if ( this.numCollisions > 0 ){
			this.steerAvoid.setMagnitude(this.velocity.magnitude());
			//this.steerAvoid.subtract(this.velocity);
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
	this.predictionVecA.setMagnitude(maxVelocity);
	this.predictionVecA.subtract(this.velocity);
	this.predictionVecA.maxLimit(maxSteering);
	this.predictionVecA.scale(ostacleStrength);
	this.accVec.add(this.predictionVecA);
}
// Return vector to another boid
boid.prototype.vecTo = function( subject ){
	return new vec(subject.position.x-this.position.x, subject.position.y-this.position.y);
}
// Check if future velocity is in obstacle
boid.prototype.circleTestA = function(range){
	for( var m = 0; m < numObstacles; m++ ){
		var vecToObstacle = new vec(0,0);
		vecToObstacle.assign(Obstacles[m].centre);
		vecToObstacle.subtract(this.position);
		if ( vecToObstacle.dotProduct(vecToObstacle) < square(Obstacles[m].radius + range + 5) ) {
			var predictedToObstacle = this.predictedPosition.squareDistance(Obstacles[m].centre);
			if ( predictedToObstacle <= square(Obstacles[m].radius + 5) ){ return true; }
			// This algorithm checks if the boids predicted position intersects an obstacle using area of triangles
			var testArea =  range * (Obstacles[m].radius + 4);
			var predictedArea = this.predictionVec.crossProduct(vecToObstacle);
			if ( predictedArea <= testArea && predictedArea!= 0 && vecToObstacle.dotProduct(vecToObstacle) > 0){ return true; }
		}
	}
	return false;
}

// Checks if Boids predicted trajectory intersects with circuar obstacle by solving quadratic equation
boid.prototype.circleTestB = function(range){
	for( var m = 0; m < numObstacles; m++ ){
		var squareToObstacle = this.position.squareDistance(Obstacles[m].centre);
		var predictedToObstacle = this.predictedPosition.squareDistance(Obstacles[m].centre);
		if ( predictedToObstacle < square(Obstacles[m].radius + 5) ){ return true; }
		else if ( squareToObstacle < square(Obstacles[m].radius + range + 5) ) {
			var temp = new vec(0,0);
			temp.assign(this.position);
			temp.subtract(Obstacles[m].centre);
			var det = square(this.predictionVec.dotProduct(temp))-this.predictionVec.dotProduct(this.predictionVec)*(temp.dotProduct(temp)-square(Obstacles[m].radius+5));
			if ( det => 0 ) {
				var quadPlus  = 0.5*(-2*this.predictionVec.dotProduct(temp)+2*Math.sqrt(det))/this.predictionVec.dotProduct(this.predictionVec);
				var quadMinus = 0.5*(-2*this.predictionVec.dotProduct(temp)-2*Math.sqrt(det))/this.predictionVec.dotProduct(this.predictionVec);
				if ( quadPlus < range && quadPlus > 0 ){return true;}
				else if ( quadMinus < range && quadPlus > 0 ){return true;}
			}
		}
	}
	return false;
}