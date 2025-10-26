#include <ros/ros.h>
#include <iostream>
#include <limits>

#include "contact_manager/AddContact.h"
#include "contact_manager/DeleteContact.h"
#include "contact_manager/GetAllContacts.h"
#include "contact_msgs/Contact.h"

void printMenu() {
    std::cout << "\n=== Contact Client Menu ===\n";
    std::cout << "1) Add Contact\n";
    std::cout << "2) Delete Contact\n";
    std::cout << "3) Get All Contacts\n";
    std::cout << "0) Exit\n";
    std::cout << "Choose: ";
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "contact_client");
    ros::NodeHandle nh;

    ros::ServiceClient add_cli  = nh.serviceClient<contact_manager::AddContact>("add_contact");
    ros::ServiceClient del_cli  = nh.serviceClient<contact_manager::DeleteContact>("delete_contact");
    ros::ServiceClient list_cli = nh.serviceClient<contact_manager::GetAllContacts>("get_all_contacts");


    add_cli.waitForExistence();
    del_cli.waitForExistence();
    list_cli.waitForExistence();

    int choice = -1;
    while (ros::ok()) {
        printMenu();
        if (!(std::cin >> choice)) {
            std::cin.clear();
            std::cin.ignore(std::numeric_limits<std::streamsize>::max(),'\n');
            continue;
        }
        std::cin.ignore(std::numeric_limits<std::streamsize>::max(),'\n');

        if (choice == 0) break;

        if (choice == 1) {

            std::string name, phone, email;
            std::cout << "Name : ";  std::getline(std::cin, name);
            std::cout << "Phone: ";  std::getline(std::cin, phone);
            std::cout << "Email: ";  std::getline(std::cin, email);

            contact_manager::AddContact srv;
            srv.request.contact.name  = name;
            srv.request.contact.phone = phone;
            srv.request.contact.email = email;

            if (add_cli.call(srv)) {
                std::cout << (srv.response.success ? "[OK] " : "[ERR] ") << srv.response.message << "\n";
            } else {
                std::cout << "[ERR] service add_contact call failed\n";
            }
        } else if (choice == 2) {

            std::string name;
            std::cout << "Name to delete: ";
            std::getline(std::cin, name);

            contact_manager::DeleteContact srv;
            srv.request.name = name;

            if (del_cli.call(srv)) {
                std::cout << (srv.response.success ? "[OK] " : "[ERR] ") << srv.response.message << "\n";
            } else {
                std::cout << "[ERR] service delete_contact call failed\n";
            }
        } else if (choice == 3) {

            contact_manager::GetAllContacts srv;
            if (list_cli.call(srv)) {
                std::cout << "Contacts (" << srv.response.contacts.size() << "):\n";
                for (const auto& c : srv.response.contacts) {
                    std::cout << " - " << c.name << " | " << c.phone << " | " << c.email << "\n";
                }
            } else {
                std::cout << "[ERR] service get_all_contacts call failed\n";
            }
        } else {
            std::cout << "Invalid choice.\n";
        }
    }
    return 0;
}
