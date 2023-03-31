// include Hyper Text Transfer Protocol (HTTP) module
var http = require('http');

// Create Server
http.createServer(function (req, res) {
  res.writeHead(200, {'Content-Type': 'text/html'});
  res.end('Hello World!');  // display "Hello World"
}).listen(8080); 