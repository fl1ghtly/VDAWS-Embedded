
# 1. Install Node.js

sudo apt update && sudo apt install -y nodejs npm

# 2. Copy server.js

mkdir relay-server

cd relay-server

nano server.js

# 3. Install dependencies

npm install express ws

# 4. Start the server on Port 80 using PM2

sudo npm install -g pm2
sudo pm2 start server.js
sudo pm2 startup && sudo pm2 save
