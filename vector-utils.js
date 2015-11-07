'use strict';
// **************** Vector Class **************
// A 2-D vector class and set of operations for vector mathematics 

// Vector constructor
function vec(x,y){
	this.x = x;
	this.y = y;
}
// Vector Reset to null vector
vec.prototype.reset = function (){
	this.x = 0; 
	this.y = 0;
}
// Vector sum function
vec.prototype.add = function (subject){
	this.x += subject.x;
	this.y += subject.y;
}
// Vector subtract function
vec.prototype.subtract = function (subject){
	this.x -= subject.x;
	this.y -= subject.y;
}
// Square Distance between points
vec.prototype.squareDistance = function (subject){
	return (this.x - subject.x)*(this.x - subject.x) + (this.y - subject.y)*(this.y - subject.y);
}
// Dot product function between vectors
vec.prototype.dotProduct = function (subject){
	return this.x*subject.x + this.y*subject.y;
}
// Cosine of angle between Vectors
vec.prototype.cosAngle = function (subject){
	return this.dotProduct(subject)/(this.magnitude() * subject.magnitude());
}
// Add vector to a cumilative average (used for boid calculation
vec.prototype.cumAvg = function(subject,count){
	this.x = this.x*(count-1)/count + subject.x/count;
	this.y = this.y*(count-1)/count + subject.y/count;
}
// Vector copyer without changing original object
vec.prototype.assign = function(subject){
	this.x = subject.x;
	this.y = subject.y;
}
// Scaling components by a scalar value
vec.prototype.scale = function(a){
	this.x = a*this.x;
	this.y = a*this.y;
}
// Return magnitude of vector
vec.prototype.magnitude = function(){
	return Math.sqrt( this.dotProduct(this) );
}
// Set magnitude of vector
vec.prototype.setMagnitude = function(size){
	this.scale(size/this.magnitude());
}
// Limit the magntude of the vector if greater than a given value
vec.prototype.maxLimit = function(maximum){
	if (this.magnitude() > maximum){ this.setMagnitude(maximum); }
}
// Apply roation matrix to vector
vec.prototype.rotate = function(angle){
	var xPrime = Math.cos(angle)*this.x - Math.sin(angle)*this.y;
	var yPrime = Math.sin(angle)*this.x + Math.cos(angle)*this.y;
	this.x = xPrime;
	this.y = yPrime;
}
// Check if the vector falls outside the boundary of the canvas
vec.prototype.boundaryDetect = function(){
	if ( this.x > xWidth-5 || this.x < 5 || this.y > yWidth-5 || this.y < 5 ){
		return true;
	}
	return false;
}
// Check if the vector is inside a round obstacle, return true if it is
vec.prototype.obstacleDetect = function(offest){
	for( var m = 0; m < numObstacles; m++ ){
		if ( this.squareDistance(Obstacles[m].centre) < square(Obstacles[m].radius + offest) ){
			return true;
		}
	}
	return false;
}