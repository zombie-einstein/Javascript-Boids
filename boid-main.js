'use strict';

// ***** Variables *****

// Canvas Elements
var canvas = document.getElementById("myCanvas");
var ctx = canvas.getContext("2d");
ctx.canvas.width  = window.innerWidth;
ctx.canvas.height = window.innerHeight; //-100;
var xWidth = canvas.width;
var yWidth = canvas.height;

// Number of elements
var numObstacles = 2; //document.getElementById("obstacles").value;	// Number of circular obstacles on canvas
var maxFormValue = 50; //document.getElementById("obstacles").max;
var numBoids = 60;		// Number of Boids

// Boid controls
var maxVelocity = 2;		// Max velocity of the Boids
var maxSteering = 0.02;		// The maximum steering force allowed
var detectionRange = 50;	// Range at which Boids become neigbours
var collisionRange = 25;	// Range at which boid will be avoided
var detectAngle = 20;		// Number of test vector angles for collision algorith
var detectionAngle = Math.cos(3 * Math.PI / 4); // Vision cone angle
var cohesionStrength = 1;
var alignStrength = 1;
var avoidStrength = 5;
var ostacleStrength = 15;

// Initialise arrays and variables
var Boids = [];
var Obstacles = [];
var myVar;
var stopped = true;
var started = false;

// Display initial boids and obstacles on loading page
resetBoids();

// ************ Functions ******************

// Square of number
function square(x){return x*x;}

// Random initial velocity
function randomVelocity(){
	var x = ( Math.random()*2 - 1 ) * maxVelocity;
	var y = ( Math.random()*2 - 1 ) * Math.sqrt( square(maxVelocity) - square(x)); // Ensures total velocity is less than max
	return new vec(x,y);
}

// Random Initial position inside canvas
function randomPosition(){
	var x = Math.random()*(xWidth-1) +1;
	var y = Math.random()*(yWidth-1) +1; 
	return new vec(x,y);
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
					if ( d < square(collisionRange) && d > 0 ){ Boids[n].collisionAvgs(Boids[m],d); }
				}
				// Check if neighbour is inside vision cone
				if ( Boids[m].velocity.cosAngle(Boids[m].vecTo(Boids[n])) >  detectionAngle){
					Boids[m].neighbourAvgs(Boids[n]);
					if ( d < square(collisionRange) && d > 0 ){ Boids[m].collisionAvgs(Boids[n],d); }
				}
	}}}}

// Animation function
function animate_b(){
	ctx.clearRect(0,0,xWidth,yWidth);
	for( var n = 0; n < numObstacles; n++){ Obstacles[n].render(); }
	for( var n = 0; n < numBoids; n++ ){ Boids[n].reset(); }
	neighbourTest();
	//document.getElementById("number").innerHTML = Boids[0].numNeighbours;
	for( var n = 0; n < numBoids; n++ ){
		if ( Boids[n].alive == true ){
			Boids[n].acceleration();
			Boids[n].move();
		}
		Boids[n].render();
	}
}

// Reset Boids to random initial positions and velocities
function resetBoids(){
	ctx.clearRect(0,0,xWidth,yWidth);
	makeObstacles();
	for( var n = 0; n < numObstacles; n++){Obstacles[n].render();}
	for( var n = 0; n < numBoids; n++ ){
		Boids[n]= new boid(randomPosition().x,randomPosition().y,randomVelocity().x,randomVelocity().y);
		// Check if boid is inside an obstacle
		// Re-roll position if it is
		while (Boids[n].position.obstacleDetect(collisionRange) === true){Boids[n].randomize();}
		Boids[n].render();
	}
}

// Run simulation function
function startSim(){ 
	if( started == false ){ 
	myVar = setInterval(animate_b, 25); 
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
	var temp = new boid(mousePos.x,mousePos.y,randomVelocity().x,randomVelocity().y);
	Boids.push(temp);
	Boids[Boids.length-1].render();
	numBoids++;
}

// Mouse position relative to canvas
function getMousePos(canvas, evt) {
        var rect = canvas.getBoundingClientRect();
	var mousePos = new vec(evt.clientX - rect.left,evt.clientY - rect.top);
	return mousePos;
}

// Generate random circular obstacles for boids
function makeObstacles() {
	for (var n = 0; n < numObstacles; n++){
		Obstacles[n] = new obstacle(Math.random()*80+20, randomPosition().x, randomPosition().y);
	}}
	
// If the window is resized, resize the canvas and shift all the elements
function resizeFunction() {
	var xScaling = window.innerWidth / canvas.width ;
	var yScaling = ( window.innerHeight-100 ) / canvas.height ;
	ctx.canvas.width  = window.innerWidth;
	ctx.canvas.height = window.innerHeight;//-100;
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

// Change the number of obstacles from HTML input
function changeObstacles (plusMinus){
	if ( numObstacles >  10){ return; }
	else{	
		numObstacles += plusMinus;	
		resetBoids();
	}
}

// Color hue changer -- Taken from "Pimp Trizkit" on Stackflow
function shadeColor2(color, percent) {   
    var f=parseInt(color.slice(1),16),t=percent<0?0:255,p=percent<0?percent*-1:percent,R=f>>16,G=f>>8&0x00FF,B=f&0x0000FF;
    return "#"+(0x1000000+(Math.round((t-R)*p)+R)*0x10000+(Math.round((t-G)*p)+G)*0x100+(Math.round((t-B)*p)+B)).toString(16).slice(1);
}