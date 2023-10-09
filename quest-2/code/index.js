// This code takes a CSV file and creates an array which it passes to an html file for plotting
// in the webpage of the "NodeServerHostIP:8080" (port 8080)

var express = require('express');
var app = express();
var path = require('path');
var fs = require('fs');
var csv = require("csv-parse");

// Import the Serial Port Modules
const { SerialPort, ReadlineParser } = require('serialport')

// Create a Port (COM7 for ESP32)
const port = new SerialPort({path: '/dev/cu.usbserial-02655165', baudRate: 115200,})
// Create a Parser
const parser = port.pipe(new ReadlineParser())

// Print to Console (see in terminal)
parser.on('data', console.log)


let data_str = ""
parser.on('data', (line) => {
  data_str = line
  // Append line to CSV file
  fs.appendFile('Serial_Output1.csv', data_str, 'utf8', function(error) {
    // In case of a error throw err.
    if (error) throw error;
    console.log('File Appended!');
  })
})

app.use(function (req, res, next) {
  res.setHeader('Access-Control-Allow-Origin', 'http://localhost:8080');
  res.setHeader('Access-Control-Allow-Methods', 'GET, POST, OPTIONS, PUT, PATCH, DELETE');
  res.setHeader('Access-Control-Allow-Headers', 'X-Requested-With,content-type');
  res.setHeader('Access-Control-Allow-Credentials', true);
  next();
  });

// viewed at http://localhost:8080/
app.get('/', function(req, res) {
  console.log("TEST");
  res.sendFile(path.join(__dirname + '/displayGraphs.html'));

});

// request data at http://localhost:8080/data or just "/data"
app.get('/data', function(req, res) {

  var data = [];  // Array to hold all csv data
  fs.createReadStream('Serial_Output1.csv')
  .pipe(csv.parse())      // fixed by using csv.parse() instead of csv()
  .on('data', (row) => {
    console.log(row);
    data.push(row);  // Add row of data to array
  })
  .on('end', () => {
    console.log('CSV file successfully processed');
    res.send(data);  // Send array of data back to requestor
  });
});

app.listen(8080);