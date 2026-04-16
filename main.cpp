#include <iostream>
#include <vector>
#include "geometry.hpp"
#include "engine.hpp"
#include "types.hpp"

#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>

enum Role { ALICE_SERVER, BOB_CLIENT };

int main(int argc, char* argv[]) {
    std::string targetIP = "127.0.0.1"; 
    int port = 8080;

    std::string role;
    if (argc > 1) {
        role = argv[1];
    }

    Role monRole;
    if(role == "server"){
        monRole = ALICE_SERVER;
    }else if(role == "client"){
        monRole = BOB_CLIENT;
    }else{
        std::cout << "Veuillez choisir un role : server ou bob" << std::endl;
        return 0;
    }

    if (monRole == ALICE_SERVER) {
        int server_fd, new_socket;
        struct sockaddr_in address;
        int opt = 1;
        int addrlen = sizeof(address);

        if ((server_fd = socket(AF_INET, SOCK_STREAM, 0)) == 0) {
            perror("socket failed");
            exit(EXIT_FAILURE);
        }

        setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT, &opt, sizeof(opt));

        address.sin_family = AF_INET;
        address.sin_addr.s_addr = INADDR_ANY;
        address.sin_port = htons(port);

        if (bind(server_fd, (struct sockaddr *)&address, sizeof(address)) < 0) {
            perror("bind failed");
            exit(EXIT_FAILURE);
        }
        
        if (listen(server_fd, 3) < 0) {
            perror("listen");
            exit(EXIT_FAILURE);
        }

        std::cout << "[ALICE] En attente de connexion sur le port " << port << "..." << std::endl;
        new_socket = accept(server_fd, (struct sockaddr *)&address, (socklen_t*)&addrlen);
        
        if (new_socket < 0) {
            perror("accept");
            return -1;
        }
        
        std::cout << "[ALICE] Drone connecte ! Réception des données..." << std::endl;

        // --- RÉCEPTION DE LA TAILLE ---
        int received = 0;
        int len = 0;
        while(received < (int)sizeof(int)){
            int res = read(new_socket, &len + received, sizeof(int) - received);
            if (res <= 0) { 
                std::cerr << "[ALICE] Erreur : Bob a coupé la connexion pendant la taille." << std::endl;
                close(new_socket); close(server_fd); return -1; 
            }
            received += res;
        }

        std::cout << "[ALICE] Taille annoncée par Bob : " << len << std::endl;

        // --- RÉCEPTION DU MESSAGE ---
        std::string message(len, 0);
        received = 0;
        while(received < len){
            int res = read(new_socket, &message[0] + received, len - received);
            received += res;
        }

        std::cout << "Message reçu : " << message << std::endl;

        close(new_socket);
        close(server_fd);
    
    } else {
        int sock = 0;
        struct sockaddr_in serv_addr;

        if ((sock = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
            return -1;
        }

        serv_addr.sin_family = AF_INET;
        serv_addr.sin_port = htons(port);
        inet_pton(AF_INET, targetIP.c_str(), &serv_addr.sin_addr);

        if (connect(sock, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0) {
            std::cerr << "[BOB] Erreur de connexion au serveur" << std::endl;
            return -1;
        }

        std::cout << "[BOB] Connecté à Alice. Envoi du message test..." << std::endl;

        std::string message = "test";
        int len = message.size();
        
        // --- ENVOI DE LA TAILLE ---
        int sent = 0;
        while(sent < (int)sizeof(int)){
            int res = write(sock, &len + sent, sizeof(int) - sent);
            sent += res;
        }

        // --- ENVOI DU MESSAGE ---
        sent = 0;
        while(sent < len){
            int res = write(sock, &message[0] + sent, len - sent);
            sent += res;
        }
        
        std::cout << "[BOB] Message envoyé !" << std::endl;
        close(sock);
    }    
    return 0;
}