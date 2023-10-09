/*
  Maxwell Bakalos, Miguel Ianus-Valdiva, Emre Karabay, Madeline Hain
  EC444 Smart & Connected Systems
  Quest 6 - Rally -- racing a course with checkpoints
  UDP Server For Communicating Between Car & Webpage
  5/2/2023
*/

// Required module
var dgram = require('dgram');
// Port and IP
var PORT_CLIENT = 3333;          // Port where the thermistor data comes from
var HOST = '192.168.1.6'; // Server Host Address (whatever is running this node server)
const PORT = process.env.PORT || 8080;  // Port where server is running from "NodeServerHostIP:PORT"
// Create socket
var server = dgram.createSocket('udp4');

// Read CSV File & Send Array to HTML File
var express = require('express');
var app = express();
var path = require('path');
// var csv = require("csv-parse");
// Create CSV File
// var fs = require('fs');
// Button for LED
// const bodyParser = require('body-parser');

// const drive_control_send_interval = 500;    // interval (milliseconds) between sending driving commands
var server_send_message = "Ok!"   // Message Sent to ESP32


/////////////////////////////////////////////////////////////
//  Receive Steering & Throttle Signal from HTML Buttons  //
////////////////////////////////////////////////////////////

// // Listen for Button Pressed Signal from HTML Page //////////
// app.use(bodyParser.urlencoded({ extended: true })); 

// Direction Signals
app.post('/leftButton', (req, res) => {
    console.log("Left")
    server_send_message = "Left"
});
app.post('/rightButton', (req, res) => {
    console.log("Right")
    server_send_message = "Right"
});
app.post('/forwardButton', (req, res) => {
    console.log("Forward")
    server_send_message = "Forward"
});
app.post('/reverseButton', (req, res) => {
    console.log("Reverse")
    server_send_message = "Reverse"
});

// Speed Signals
app.post('/stopButton', (req, res) => {
    console.log("Stop")
    server_send_message = "Stop"
});
app.post('/startButton', (req, res) => {
    console.log("Start")
    server_send_message = "Start"
});
app.post('/slowButton', (req, res) => {
    console.log("Slow")
    server_send_message = "Slow"
});
app.post('/mediumButton', (req, res) => {
    console.log("Medium")
    server_send_message = "Medium"
});
app.post('/fastButton', (req, res) => {
    console.log("Fast")
    server_send_message = "Fast"
});

// On connection, print out received message
server.on('message', function (message, remote) {
    console.log(remote.address + ':' + remote.port +' - ' + message);

    // Send Driving Control Message
    server.send(server_send_message, remote.port, remote.address, function(error){
        if(error){
          console.log('Driving Control Send Error!');
        }
        else{
          console.log('Sent: ' + server_send_message);
        }
    });
});


/////////////////////
//  Server Stuff  //
////////////////////

// Create server that listens on a port
server.on('listening', function () {
    var address = server.address();
    console.log('UDP Server listening on ' + address.address + ":" + address.port);
});

// // Bind server to port and IP
// server.bind({
//     address: HOST,
//     port: PORT_CLIENT,
//     // exclusive: true
// });
// Bind server to port and IP
server.bind(PORT_CLIENT, HOST);

// viewed at http://localhost:8080
app.get('/', function(req, res) {
    res.sendFile(path.join(__dirname + '/Q6_browser.html')); // <-- HTML File for Webpage
});

// Serve static files from the 'public' directory (for jQuery)
app.use(express.static('jQuery_File'))

// Errors
server.on('error', (err) => {
    console.log(`server error:\n${err.stack}`);
    server.close();
});

// Create server that listens on a port
server.on('listening', function () {
  var address = server.address();
  console.log('UDP Server listening on ' + address.address + ":" + address.port);
});

// Start the Server
app.listen(PORT, () => {
    console.log(`Server is running on port ${PORT}`);
});

// // Function to Periodically Send out Driving Control Commands
// function send_drive_control() {
//     // Send Driving Control Message
//     server.send(server_send_message, PORT_CLIENT, remote.address, function(error){
//         if(error){
//           console.log('Driving Control Send Error!');
//         }
//         else{
//           console.log('Sent: ' + server_send_message);
//         }
//     });
// }
// setInterval(send_drive_control, drive_control_send_interval);


// // On connection, print out received message
// server.on('message', function (message, remote) {
//     console.log(remote.address + ':' + remote.port +' - ' + message);

//     // Send Driving Control Message
//     server.send(server_send_message, PORT_CLIENT, remote.address, function(error){
//         if(error){
//           console.log('Driving Control Send Error!');
//         }
//         else{
//           console.log('Sent: ' + server_send_message);
//         }
//     });
// });



