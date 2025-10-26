#ifndef CONTACT_HPP
#define CONTACT_HPP

#include <string>
#include <iostream>

class Contact {
private:
    std::string name;
    std::string phone;
    std::string email;

public:
    Contact() = default;
    Contact(const std::string& name, const std::string& phone, const std::string& email)
        : name(name), phone(phone), email(email) {}

    const std::string& getName()  const { return name;  }
    const std::string& getPhone() const { return phone; }
    const std::string& getEmail() const { return email; }

    void setPhone(const std::string& p) { phone = p; }
    void setEmail(const std::string& e) { email = e; }

    void display() const {
        std::cout << "Name: " << name
                  << ", Phone: " << phone
                  << ", Email: " << email << std::endl;
    }

    bool operator==(const std::string& otherName) const {
        return name == otherName;
    }
};

#endif
