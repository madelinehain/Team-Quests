var fs = require('fs');

// "Create a new file using the appendFile() method:"
fs.appendFile('mynewfile1.txt', 'Hello content!', function (err) {
  if (err) throw err;
  console.log('Saved!');
}); 

/* "The fs.open() method takes a "flag" as the second argument, 
if the flag is "w" for "writing", the specified file is opened for writing. If the file does not exist, an empty file is created:" */
// Create a new, empty file using the open() method:
fs.open('mynewfile2.txt', 'w', function (err, file) {
  if (err) throw err;
  console.log('Saved!');
}); 

/* "The fs.writeFile() method replaces the specified file and content if it exists. If the file does not exist, a new file, 
containing the specified content, will be created:" */
// Create a new file using the writeFile() method:
fs.writeFile('mynewfile3.txt', 'Hello content!', function (err) {
  if (err) throw err;
  console.log('Saved!');
}); 