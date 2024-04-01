# F1Tenth Lab 1: Introduction to docker and ros

## Outline
This lab is an introduction to docker and ros. Its goal is to get familiar with the concepts of publishers, subscribers, launch files and docker.

## Requirements
- Docker Desktop

## Running the Lab
To run the lab, perform the following steps:

1. Clone the repository to a local directory

2. Open **Docker Desktop**

3. Open a command prompt and naviagte to the repository directory (Dockerfile directory)

4. Run the following command in the command prompt to build the docker image. 
```
docker build . -t f1tenth:lab1
```

5. Run the following command in the command prompt to start the docker container and see the nodes communicating
```
docker run -it --name container_name f1tenth:lab#
```

## Further Development
To modify the code, simply open the lab folder locally in **VSCode** and perform the changes. Then perform steps 3 through 5 to run the code. The dockerfile can be modified to stop the nodes from launching automatically. This would be needed to develop and debug in the command prompt with ros commands.