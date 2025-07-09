#include <iostream>
#include <vector>
#include <string>
#include <algorithm>  // for std::reverse
#include "urdf/model.h"         // [src/model.cpp](src/model.cpp)
#include "urdf/link.h"          // [include/urdf/link.h](include/urdf/link.h)

using namespace urdf;

void accumulateChainTransforms(std::shared_ptr<Link> link,
                               const std::string& base,
                               std::vector<std::shared_ptr<Link>>& chain) {
    while (link && link->name != base) {
        chain.push_back(link);
        link = link->getParent();
    }
    if (link && link->name == base) {
        chain.push_back(link);
    }
}

int main(int argc, char** argv) {
    if (argc < 4) {
        std::cerr << "Usage: " << argv[0] << " <urdf_file> <base_link> <ee_link>\n";
        return 1;
    }

    std::string urdf_file = argv[1];
    std::string base_link_name = argv[2];
    std::string ee_link_name   = argv[3];

    // Load URDF
    std::shared_ptr<UrdfModel> model;
    try {
        model = UrdfModel::fromUrdfFile(urdf_file.c_str());  // [src/model.cpp](src/model.cpp)
    } catch (...) {
        std::cerr << "Failed to load URDF\n";
        return 1;
    }

    // Get end-effector link
    auto ee_link = model->getLink(ee_link_name);
    if (!ee_link) {
        std::cerr << "EE link not found\n";
        return 1;
    }

    // Build chain from EE back to base
    std::vector<std::shared_ptr<Link>> chain;
    accumulateChainTransforms(ee_link, base_link_name, chain);

    // Compute transforms and inertial values in each link's frame
    std::cout << "Chain from " << base_link_name << " to " << ee_link_name << ":\n";
    std::cout << "=========================================================\n\n";
    
    // Initialize cumulative transform (identity transform)
    Transform base_to_current;  // Identity transform
    Transform previous_to_current; // For relative transforms
    
    // Reverse the chain to go from base to end-effector
    std::reverse(chain.begin(), chain.end());
    
    for (size_t i = 0; i < chain.size(); ++i) {
        auto& link = chain[i];
        std::cout << "Link " << (i+1) << "/" << chain.size() << ": " << link->name << "\n";
        std::cout << "-----------------------------------------\n";
        
        // Print cumulative transform from base to current link
        std::cout << "  Cumulative transform from " << base_link_name << " to " << link->name << ":\n";
        std::cout << "    Position (xyz): "
                  << base_to_current.position().x()<< " "
                  << base_to_current.position().y()<< " "
                  << base_to_current.position().z()<< "\n";
        double roll, pitch, yaw;
        base_to_current.rotation().getRpy(roll, pitch, yaw);
        std::cout << "    Rotation (rpy): " << roll << " " << pitch << " " << yaw << "\n";
        
        // Print relative transform from previous link (if not the first link)
        if (i > 0) {
            std::cout << "  Relative transform from " << chain[i-1]->name << " to " << link->name << ":\n";
            std::cout << "    Position (xyz): "
                      << previous_to_current.position().x()<< " "
                      << previous_to_current.position().y()<< " "
                      << previous_to_current.position().z()<< "\n";
            previous_to_current.rotation().getRpy(roll, pitch, yaw);
            std::cout << "    Rotation (rpy): " << roll << " " << pitch << " " << yaw << "\n";
        }
        
        if (link->inertial.has_value()) {
            auto& inert = link->inertial.value();
            std::cout << "  Inertial properties:\n";
            std::cout << "    Mass: " << inert.mass << "\n";
            std::cout << "    COM offset (xyz): "
                      << inert.origin.position().x()<< " "
                      << inert.origin.position().y()<< " "
                      << inert.origin.position().z()<< "\n";
        }
        
        // Update cumulative transform for next link
        // Find the joint that connects this link to its child
        if (!link->child_joints.empty()) {
            auto joint = link->child_joints[0]; // Assuming single chain
            std::cout << "  Outgoing joint: " << joint->name << "\n";
            std::cout << "    Joint transform (xyz): "
                      << joint->parent_to_joint_transform.position().x()<< " "
                      << joint->parent_to_joint_transform.position().y()<< " "
                      << joint->parent_to_joint_transform.position().z()<< "\n";
            joint->parent_to_joint_transform.rotation().getRpy(roll, pitch, yaw);
            std::cout << "    Joint transform (rpy): " << roll << " " << pitch << " " << yaw << "\n";
            
            // Store the joint transform as the relative transform for the next iteration
            previous_to_current = joint->parent_to_joint_transform;
            
            // Accumulate the joint transform
            base_to_current = base_to_current * joint->parent_to_joint_transform;
            std::cout << "  Cumulative transform updated.\n";
        } else {
            std::cout << "  (End-effector - no outgoing joints)\n";
        }
        std::cout << "\n";
    }

    return 0;
}