In order to run openMCT we must first have nodejs and npm installed.  Due to a namespace conflict (described here: http://stackoverflow.com/questions/21168141/cannot-install-packages-using-node-package-manager-in-ubuntu) we must install the legacy version of nodejs (even though the current nodejs is already installed).  Run the following commands:

sudo apt-get install npm			#installs npm for us

sudo apt-get install nodejs-legacy		#installs legacy nodejs for us

npm install 					#builds openMCT project

npm start					#starts openMCT


						#after running ‘nmp start’ you should be able to view openMCT by opening a browser and going to localhost:8080
