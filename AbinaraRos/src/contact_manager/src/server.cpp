#include <ros/ros.h>
#include "contact_manager/ContactManager.hpp"

#include "contact_manager/AddContact.h"
#include "contact_manager/DeleteContact.h"
#include "contact_manager/GetAllContacts.h"

#include "contact_msgs/Contact.h"

class ContactServerNode {
private:
    ros::NodeHandle nh_;
    ContactManager manager_;
    ros::ServiceServer srv_add_;
    ros::ServiceServer srv_del_;
    ros::ServiceServer srv_getall_;

public:
    ContactServerNode() {
        srv_add_   = nh_.advertiseService("add_contact",   &ContactServerNode::onAdd,   this);
        srv_del_   = nh_.advertiseService("delete_contact",&ContactServerNode::onDelete,this);
        srv_getall_= nh_.advertiseService("get_all_contacts",&ContactServerNode::onGetAll,this);
        ROS_INFO("Contact Server is ready.");
    }

    bool onAdd(contact_manager::AddContact::Request &req,
               contact_manager::AddContact::Response &res) {
        Contact c(req.contact.name, req.contact.phone, req.contact.email);
        bool ok = manager_.addContact(c);
        res.success = ok;
        res.message = ok ? "Contact added successfully" : "Contact already exists";
        return true;
    }

    bool onDelete(contact_manager::DeleteContact::Request &req,
                  contact_manager::DeleteContact::Response &res) {
        bool ok = manager_.deleteContact(req.name);
        res.success = ok;
        res.message = ok ? "Contact deleted" : "Contact not found";
        return true;
    }

    bool onGetAll(contact_manager::GetAllContacts::Request & /*req*/,
                  contact_manager::GetAllContacts::Response &res) {
        auto all = manager_.getAll();
        res.contacts.reserve(all.size());
        for (const auto& c : all) {
            contact_msgs::Contact mc;
            mc.name  = c.getName();
            mc.phone = c.getPhone();
            mc.email = c.getEmail();
            res.contacts.push_back(mc);
        }
        return true;
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "contact_server");
    ContactServerNode server;
    ros::spin();
    return 0;
}
