// Use the module "myfirstmodule" in a Node.js file:

// include Hyper Text Transfer Protocol (HTTP) module
var http = require('http');
var dt = require('./myfirstmodule');    // look for the module in the same folder this file is in

// Create Server:
/* "The function passed into the http.createServer() method, will be 
executed when someone tries to access the computer on port 8080." */
http.createServer(function (req, res) {
  /* "The first argument of the res.writeHead() method is the status code, 
  200 means that all is OK, the second argument is an object containing 
  the response headers." */
  res.writeHead(200, {'Content-Type': 'text/html'});
  res.write("The date and time are currently: " + dt.myDateTime()); // write
  res.end();
}).listen(8080); 