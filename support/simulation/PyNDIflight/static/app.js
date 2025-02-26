/**
 * Drone visuals with THREE.js, parsing craft data from an HTTP endpoint
 * 
 * Copyright 2024 Till Blaha (Delft University of Technology)
 *
 * This program is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program. If not, see <https://www.gnu.org/licenses/>.
 */

import * as THREE from "three";
import { OrbitControls } from "three/addons/controls/OrbitControls.js";
import { ViewHelper } from "three/addons/helpers/ViewHelper.js";
import Stats from "stats.js";
import {
    craftTypes,
    quadRotor,
    tailSitter,
    airplane,
    quadRotorExternal,
    } from "./crafts.js";

var clock = new THREE.Clock();

async function updateVisualization(data) {
    for (let i = 0; i < data.length; i++) {
        var existsAlready = idList.includes(data[i].id);
        var shouldReload = data[i].newCraft;
        if (!existsAlready || shouldReload) {
            // never seen this id, add new craft to scene
            var newCraft;
            switch (data[i].type) {
                case craftTypes.quadRotor:
                    newCraft = new quadRotor(0.12, 0.09, 3.*0.0254);
                    break;
                case craftTypes.tailSitter:
                    newCraft = new tailSitter(1., 0.3, 6*0.0254);
                    break;
                case craftTypes.airplane:
                    newCraft = new airplane(1., 0.3, 8*0.0254);
                    break;
                case craftTypes.quadRotorExternal:
                    // synchronous call because i'm lazy
                    const response = await fetch('/craftdata');
                    const json = await response.json();
                    newCraft = new quadRotorExternal(json);
                    break;
                default:
                    break;
            }

            if (!existsAlready) {
                idList.push(data[i].id);
                craftList.push(newCraft);
            } else {
                scene.remove(craftList[data[i].id].obj);
                craftList[data[i].id] = newCraft;
            }

            scene.add(newCraft.obj);
            scene.add(newCraft.line);
        }
        var idx = idList.indexOf(data[i].id);
        craftList[idx].setPose(data[i].pos, data[i].quat);
        craftList[idx].setControls(data[i].ctl);
    }
}
window.updateVisualization = updateVisualization;

function fetchPose() {
    fetch('/pose')
    //fetch('/pose_showoff')
      .then(response => {
        // Check if the request was successful (status code 200)
        if (!response.ok) {
          throw new Error('Network response was not ok');
        }
        // Parse the response as JSON
        return response.json();
      })
      .then(data => {
        // Use the parsed JSON data
        updateVisualization(data);
      })
      .catch(error => {
        // Handle errors
        console.error('There was a problem with the fetch operation:', error);
      });
}

var time_since_fetch = 0;

function animate() {
    stats.begin()

	requestAnimationFrame( animate );

    const delta = clock.getDelta();
    time_since_fetch += delta;
    if (time_since_fetch > 0.03) {
        time_since_fetch = 0;
        fetchPose();
    }
    if ( viewHelper.animating ) viewHelper.update( delta );

    renderer.clear();
	renderer.render( scene, camera );
    viewHelper.render( renderer );

    stats.end()
}

var idList = []
var craftList = []
window.craftList = craftList

// webGL renderer
const renderer = new THREE.WebGLRenderer( { antialias: true } );
renderer.setSize( window.innerWidth, window.innerHeight );
renderer.autoClear = false;
document.body.appendChild( renderer.domElement )

// scene with white background
const scene = new THREE.Scene();
scene.background = new THREE.Color(0xffffff);

//const width = 5;
//const height = 7;
//const depth = 0.1;  // Very thin depth, almost like a 2D rectangle
//
//const geometry = new THREE.BoxGeometry(width, height, depth);
//const edges = new THREE.EdgesGeometry(geometry);
//const material = new THREE.LineBasicMaterial({ color: 0x000000 });
//const gateOutline = new THREE.LineSegments(edges, material);
//
//scene.add(gateOutline);

// camera, such that North East Down makes sense
const camera = new THREE.PerspectiveCamera( 40, window.innerWidth / window.innerHeight, 0.1, 1000 );
camera.up.set( 0, 0, -1 ); // for orbit controls to make sense
camera.position.x = -10;
camera.position.y = 8;
camera.position.z = -5.0;
camera.setRotationFromEuler( new THREE.Euler(-110*3.1415/180, 0, 55 * 3.1415/180, 'ZYX'))

window.onresize = function() {
    var margin = 35;
    camera.aspect = (window.innerWidth-margin) / (window.innerHeight-margin);
    camera.updateProjectionMatrix();
    renderer.setSize(window.innerWidth-margin, window.innerHeight-margin);
};

// ground plane grid in the xy-plane and coordinate system stems
const gd = new THREE.GridHelper( 10, 10 );
gd.rotation.x = -0.5*3.1415
scene.add( gd );
scene.add( new THREE.AxesHelper ( 0.75 ));

// interactive camera controls and triad in the corner
const controls = new OrbitControls( camera, renderer.domElement );
document.addEventListener('keydown', function(event) { // reset view on space
    if (event.code === 'Space') { controls.reset(); } });
var viewHelper = new ViewHelper( camera, renderer.domElement );
viewHelper.controls = controls;
viewHelper.controls.center = controls.target;
window.onpointerup = function (event) { // enable clicking of the triad
    viewHelper.handleClick( event ) };

window.onresize() // call once

// performance counter in top left
const stats = new Stats()
stats.showPanel(0) // 0: fps, 1: ms, 2: mb, 3+: custom
document.body.appendChild(stats.dom)

animate();
