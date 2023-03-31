/*
Update Files

The File System module has methods for updating files:

    fs.appendFile()
    fs.writeFile()

The fs.appendFile() method appends the specified content at the end of the specified file:
*/


var fs = require('fs');

// Append "This is my text." to the end of the file "mynewfile1.txt":
fs.appendFile('mynewfile1.txt', ' This is my text.', function (err) {
  if (err) throw err;
  console.log('Updated!');
}); 

// The fs.writeFile() method replaces the specified file and content:
// Replace the content of the file "mynewfile3.txt":
fs.writeFile('mynewfile3.txt', 'This is my text', function (err) {
  if (err) throw err;
  console.log('Replaced!');
}); 

