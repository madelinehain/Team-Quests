/*
Using a Package

Once the package is installed, it is ready to use.

Include the "upper-case" package the same way you include any other module:
*/

// Create a Node.js file that will convert the output "Hello World!" into upper-case letters:

// go to http://localhost:8080

var http = require('http');
var uc = require('upper-case');
http.createServer(function (req, res) {
  res.writeHead(200, {'Content-Type': 'text/html'});
  res.write(uc.upperCase("Hello World!"));
  res.end();
}).listen(8080); 