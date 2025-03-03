const express = require("express");
const path = require("path");

const app = express();
const port = 3000;

// Serve the Next.js build files
app.use(express.static(path.join(__dirname, "out")));

app.listen(port, () => {
    console.log(`AUV Dashboard running at http://localhost:${port}`);
});
