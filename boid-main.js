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
function startSim(){ 
	if( started == false ){ 
	myVar = setInterval (animate_b, 25); 
	started = true; 
	stopped = false; }
	else { return; }
}
// Pause Simulation function
function pauseSim(){
	if( stopped == false ){ 
	clearInterval(myVar);
	stopped = true;
	started = false;}
}
// Add boid at mouse click
function addBoid(event){
	var mousePos = getMousePos(canvas, event);
	if (mousePos.obstacleDetect(5) == true){return;}
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
	var mousePos = new vec(evt.clientX - rect.left,evt.clientY - rect.top);
	return mousePos;
}
// Generate circular obstacles for boids
function makeObstacles() {
	for (var n = 0; n < numObstacles; n++){
		Obstacles[n] = new obstacle(Math.random()*80+20, Math.random()*(xWidth-1) +1, Math.random()*(yWidth-1) +1);
	}}
// If the window is resized, resize the canvas and shift all the elements
function resizeFunction() {
	var xScaling = window.innerWidth / canvas.width ;
	var yScaling = ( window.innerHeight-100 ) / canvas.height ;
	ctx.canvas.width  = window.innerWidth;
	ctx.canvas.height = window.innerHeight-100;
	xWidth = canvas.width;
	yWidth = canvas.height;
	for( var n = 0; n < numObstacles; n++){
		Obstacles[n].centre.x = xScaling * Obstacles[n].centre.x;
		Obstacles[n].centre.y = yScaling * Obstacles[n].centre.y;
		Obstacles[n].render();
	}
	for( var n = 0; n < numBoids; n++ ){
		Boids[n].position.x = xScaling * Boids[n].position.x;
		Boids[n].position.y = yScaling * Boids[n].position.y;
		Boids[n].render();
	}
}
// Change the number of obstacles
function changObstacles (){
	if ( document.getElementById("obstacles").value > document.getElementById("obstacles").max ){
		return; }
	else{	
		numObstacles = document.getElementById("obstacles").value;	
		resetBoids();
	}
}
//******* Running simulation starts here *************

var numBoids = 60;		// Number of Boids
var numObstacles = document.getElementById("obstacles").value;	// Number of circular obstacles on canvas
var maxVelocity = 2;		// Max velocity of the Boids
var maxSteering = 0.01;		// The maximum steering force allowed
var detectionRange = 50;	// Range at which Boids become neigbours
var collisionRange = 25;	// Range at which boid will be avoided
var detectAngle = 16;		// Number of test vector angles for collision algorith
var detectionAngle = Math.cos(3 * Math.PI / 4); 
var canvas = document.getElementById("myCanvas");
var ctx = canvas.getContext("2d");
ctx.canvas.width  = window.innerWidth;
ctx.canvas.height = window.innerHeight-100;
var xWidth = canvas.width;
var yWidth = canvas.height;
var Boids = [];
var Obstacles = [];
var myVar;
var stopped = true;
var started = false;

resetBoids();



