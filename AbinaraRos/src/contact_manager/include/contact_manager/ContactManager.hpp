#ifndef CONTACT_MANAGER_HPP
#define CONTACT_MANAGER_HPP

#include "Contact.hpp"
#include <vector>
#include <algorithm>
#include <optional>

class ContactManager {
private:
    std::vector<Contact> contacts;

public:
    bool addContact(const Contact& c) {
        auto it = std::find_if(contacts.begin(), contacts.end(),
            [&](const Contact& x){ return x.getName() == c.getName(); });
        if (it != contacts.end()) return false; // sudah ada
        contacts.emplace_back(c);
        return true;
    }

    bool deleteContact(const std::string& name) {
        auto it = std::find_if(contacts.begin(), contacts.end(),
            [&](const Contact& x){ return x.getName() == name; });
        if (it == contacts.end()) return false;
        contacts.erase(it);
        return true;
    }

    std::vector<Contact> getAll() const {
        return contacts;
    }
};

#endif
