# F1Tenth_Labs

## Outline
This repository contains the labs from the F1Tenth course.

The files for each lab are organized under a folder named **lab#_ws**, for example **lab1_ws**.

## Requirements
- Docker

## Running a Lab
To run a lab, perform the following steps:
1. Clone the repository to a local directory
2. Open the **Dockerfile** in a VSCode or a text editor, and for the argument LAB_WS, specify the folder of the lab you want to run
3. Open a command prompt and naviagte to the repository directory (Dockerfile directory)
4. Run the following command in the command prompt to build the docker image. Replace the **#** by the lab number, for example f1tenth/lab1.
```
docker build . -t f1tenth:lab#
```
5. Run the following command in the command prompt to start the docker container.
```
docker run -it --name container_name f1tenth:lab#
```