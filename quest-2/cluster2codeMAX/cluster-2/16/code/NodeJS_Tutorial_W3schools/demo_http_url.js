var http = require('http');

/* "The function passed into the http.createServer() has a req argument that 
represents the request from the client, as an object (http.IncomingMessage object)." */
http.createServer(function (req, res) {
  res.writeHead(200, {'Content-Type': 'text/html'});
  res.write(req.url);
  res.end();
}).listen(8080); 

/* 
Go to
http://localhost:8080/summer
&
http://localhost:8080/winter
& see the difference
*/