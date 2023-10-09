/*
  Maxwell Bakalos, Miguel Ianus-Valdiva, Emre Karabay, Madeline Hain
  EC444 Smart & Connected Systems
  Quest 4 - Road Trip
  UDP Server for Thermistor, Video, & LED Button
  3/31/2023
*/

// This code takes a CSV file and creates an array which it passes to an html file for plotting
// in the webpage of the "NodeServerHostIP:8080" (port 8080)

// Required module
var dgram = require('dgram');
// Port and IP
var PORT_ESP = 8080;          // Port where the thermistor data comes from
var HOST = '192.168.1.6'; // Server Host Address (whatever is running this node server)
const PORT = process.env.PORT || 8080;  // Port where server is running from "NodeServerHostIP:PORT"
// Create socket
var server = dgram.createSocket('udp4');

// Read CSV File & Send Array to HTML File
var express = require('express');
var app = express();
var path = require('path');
var csv = require("csv-parse");
// Create CSV File
var fs = require('fs');
// Button for LED
const bodyParser = require('body-parser');

var server_send_message = "Ok!"   // Message Sent to ESP32
var startStop_toggle = 0             // LED Button Toggle

// Serve static files from the 'public' directory (for jQuery)
app.use(express.static('jQuery_File'))

// Listen for Button Pressed Signal from HTML Page //////////
app.use(bodyParser.urlencoded({ extended: true })); 
// // Receive LED Toggle Signal
// app.post('/startButton', (req, res) => {

//   console.log("START/STOP PRESSED!!!")
//   if (startStop_toggle == 1) {
//     // server_send_message = "Ok!"
//     server_send_message = "STOP"
//     startStop_toggle = 0 // set buggy to STOP
//   }
//   else if (startStop_toggle == 0) {
//     // server_send_message = "BUTTON_PRESSED"
//     server_send_message = "START"
//     startStop_toggle = 1 // set buggy to START
//   }
// });

// Receive Buggy Motor Toggle Signal
app.post('/startButton', (req, res) => {

  console.log("START PRESSED!!!")
  server_send_message = "START"
  startStop_toggle = 1 // set buggy to START

});

// Receive Buggy Motor Toggle Signal
app.post('/stopButton', (req, res) => {

  console.log("STOP PRESSED!!!")
  server_send_message = "STOP"
  startStop_toggle = 0 // set buggy to STOP

});

// Receive Buggy Motor Toggle Signal
app.post('/emergencyStopButton', (req, res) => {

  console.log("EMERGENCY STOP PRESSED!!!")
  server_send_message = "EMERGENCY_STOP"
  startStop_toggle = 0 // set buggy to STOP

});



// Create server that listens on a port
server.on('listening', function () {
  var address = server.address();
  console.log('UDP Server listening on ' + address.address + ":" + address.port);
});

// // Create CSV File
// fs.open('Sensor_Output.csv', 'w', function (err, file) {
//   if (err) throw err;
//   console.log('Saved File!');
// }); 

// let data_str = ""
// On connection, print out received message
server.on('message', function (message, remote) {
  console.log(remote.address + ':' + remote.port +' - ' + message);

  // // data_str =  message + "\n"
  // data_str = message
  // // Append line to CSV file
  // fs.appendFile('Sensor_Output.csv', data_str, 'utf8', function(error) {
  //   // In case of a error throw err.
  //   if (error) throw error;
  //   console.log('File Appended!');
  // })

    // Send Ok acknowledgement
    server.send(server_send_message,remote.port,remote.address,function(error){
      if(error){
        console.log('MEH!');
      }
      else{
        console.log('Sent: ' + server_send_message);
        // server_send_message = "Ok!"
      }
    });

});

// Bind server to port and IP
server.bind(PORT_ESP, HOST);

// viewed at http://localhost:8080
app.get('/', function(req, res) {
  res.sendFile(path.join(__dirname + '/Q4_Web_Start_Stop.html')); // <-- HTML File for Webpage
});

// Start the Server
app.listen(PORT, () => {
  console.log(`Server is running on port ${PORT}`);
});
