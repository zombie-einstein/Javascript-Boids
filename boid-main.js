'use strict';

// ************ Functions ******************

// Square of number
function square(x){return x*x;}
// Neighbour check function
function neighbourTest(){
	// The arrangement of these loops means that neighbour checks are only
	// made once between pairs of boids, slightly more effecient than
	// check all boids for each boid	
	for ( var n = 0; n < numBoids-1; n++ ){
	for ( var m = n+1; m < numBoids; m++ ){ 
		// This checks that neighbours are both within a certain range
		// Check angle is more involved as it requires checking the angle in both directions
		var d = Boids[n].position.squareDistance(Boids[m].position);
		if ( d < square(detectionRange) ){
				Boids[n].neighbourAvgs(Boids[m]); 
				Boids[m].neighbourAvgs(Boids[n]);
				// Check if neighbour falls within collision angle
				if ( d < square(collisionRange) && d > 0 ){
					Boids[n].collisionAvgs(Boids[m],d); 
					Boids[m].collisionAvgs(Boids[n],d);
	}}}}}
// Animation function
function animate_b(){
	ctx.clearRect(0,0,xWidth,yWidth);
	for( var n = 0; n < numObstacles; n++){Obstacles[n].render();}
	for( var n = 0; n < numBoids; n++ ){ Boids[n].reset(); }
	neighbourTest();
	for( var n = 0; n < numBoids; n++ ){
		Boids[n].acceleration();
		Boids[n].move();
		Boids[n].render();
	}

}
// Reset Boids to random initial positions and velocities
function resetBoids(){
	ctx.clearRect(0,0,xWidth,yWidth);
	makeObstacles();
	for( var n = 0; n < numObstacles; n++){Obstacles[n].render();}
	for( var n = 0; n < numBoids; n++ ){
		Boids[n]= new boid();
		var test = 0;
		do {Boids[n].initial(); test++;}
		// Check if boid is inside an obstacle
		// Re-roll position if it is
		while (Boids[n].position.obstacleDetect(collisionRange) === true);
		Boids[n].render();
	}
}
// Run simulation function
function startSim(){ myVar = setInterval (animate_b, 25); }
// Pausi Simulation function
function pauseSim(){clearInterval(myVar);}
// Add boid at mouse click
function addBoid(event){
	var mousePos = getMousePos(c, event);
	var temp = new boid();
	Boids.push(temp);
	Boids[Boids.length-1].initial();
	Boids[Boids.length-1].position.x = mousePos.x;
	Boids[Boids.length-1].position.y = mousePos.y;
	Boids[Boids.length-1].render();
	numBoids++;
}
// Mouse position relative to canvas
function getMousePos(canvas, evt) {
        var rect = canvas.getBoundingClientRect();
        return {
          x: evt.clientX - rect.left,
          y: evt.clientY - rect.top
        };
      }
// Generate circular obstacles for boids
function makeObstacles() {
	for (var n = 0; n < numObstacles; n++){
		Obstacles[n] = new obstacle(Math.random()*80+20, Math.random()*(xWidth-1) +1, Math.random()*(yWidth-1) +1);
	}}
//******* Running simulation starts here *************

var numBoids = 60;
var numObstacles = 2;
var maxVelocity = 2;
var maxSteering = 0.05;
var detectionRange = 50;
var collisionRange = 25;
var detectionAngle = Math.cos(3 * Math.PI / 4); 
var c = document.getElementById("myCanvas");
var ctx = c.getContext("2d");
ctx.canvas.width  = window.innerWidth;
ctx.canvas.height = window.innerHeight-100;
var xWidth = c.width;
var yWidth = c.height;
var Boids = [];
var Obstacles = [];
var myVar;

resetBoids();



