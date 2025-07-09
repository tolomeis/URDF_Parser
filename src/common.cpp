#include "urdf/common.h"
#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>

using namespace urdf;
using namespace std;

// ------------------- Vector Implementation -------------------

Vector3 Vector3::fromVecStr(const string& vector_str) {
	urdf::Vector3 vec;

	vector<string> pieces;
	vector<double> values;

	boost::split( pieces, vector_str, boost::is_any_of(" "));
	for (unsigned int i = 0; i < pieces.size(); ++i){
		if (pieces[i] != ""){
			try {
				values.push_back(boost::lexical_cast<double>(pieces[i].c_str()));
			} catch (boost::bad_lexical_cast &e) {
				throw URDFParseError("Error not able to parse component (" + pieces[i] + ") to a double (while parsing a vector value)");
			}
		}
	}

	if (values.size() != 3) {
		ostringstream error_msg;
		error_msg << "Parser found " << values.size()
				  << " elements but 3 expected while parsing vector ["
				  << vector_str <<  "]";
		throw URDFParseError(error_msg.str());
	}

	vec.x() = values[0];
	vec.y() = values[1];
	vec.z() = values[2];

	return vec;
}

// ------------------- Quaternion Implementation -------------------

void Rotation::getRpy(double &roll, double &pitch, double &yaw) const {
	double sqw;
	double sqx;
	double sqy;
	double sqz;

	sqx = x() * x();
	sqy = y() * y();
	sqz = z() * z();
	sqw = w() * w();

	roll = atan2( 2*(y()*z() + w()*x()), sqw - sqx - sqy + sqz );
	double s = -2*(x()*z() - w()*y());
	if (s <= -1.) {
		pitch = -0.5*M_PI;
	} else if (s >= 1.) {
		pitch = 0.5*M_PI;
	} else {
		pitch = asin(s);
	}
	yaw   = atan2( 2*(x()*y() + w()*z()), sqw + sqx - sqy - sqz );
}

Rotation Rotation::fromRpy(double roll, double pitch, double yaw) {
	// Use Eigen's AngleAxis composition() for RPY
	Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitX());
	Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitY());
	Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitZ());
	
	Eigen::Quaterniond q = yawAngle * pitchAngle * rollAngle;
	return Rotation(q);
};


Rotation Rotation::fromRpyStr(const string &rotation_str) {
	urdf::Vector3 rpy = urdf::Vector3::fromVecStr(rotation_str);
	return Rotation::fromRpy(rpy.x(), rpy.y(), rpy.z());
}

// ------------------- Color Implementation -------------------

Color Color::fromColorStr(const std::string &vector_str) {
	std::vector<std::string> pieces;
	std::vector<float> values;

	boost::split(pieces, vector_str, boost::is_any_of(" "));
	for (unsigned int i = 0; i < pieces.size(); i++) {
		if (!pieces[i].empty()) {
			try {
				values.push_back(boost::lexical_cast<double>(pieces[i].c_str()));
			} catch (boost::bad_lexical_cast &e) {
				std::ostringstream error_msg;
				error_msg << "Error parsing Color value " << i
						  << " in color value string (" << vector_str
						  << "): value is not a double: " << e.what() << "!";
				throw URDFParseError(error_msg.str());
			}
		}
	}

	if (values.size() != 4) {
		std::ostringstream error_msg;
		error_msg << "Error parsing Color string (" << vector_str
				  << "): It needs to contain exactly 4 values for rbdl color!";
		throw URDFParseError(error_msg.str());
	}

	return Color( values[0], values[1], values[2], values[3] );
}

// ------------------- Transform Implementation -------------------

// Transform Transform::operator*(const Transform& other) const {
// 	Transform result;
// 	// Use Eigen operations for more efficient and robust transforms
// 	result.position() = Vector3(this->rotation() * static_cast<const Eigen::Vector3d&>(other.position()) + static_cast<const Eigen::Vector3d&>(this->position()));
// 	result.rotation() = Rotation(static_cast<const Eigen::Quaterniond&>(this->rotation()) * static_cast<const Eigen::Quaterniond&>(other.rotation()));
// 	return result;
// }

Transform Transform::fromXml(TiXmlElement* xml) {
	Transform t;
	if (xml) {
		const char* xyz_str = xml->Attribute("xyz");
		if (xyz_str != NULL) {
			t.setPosition(Vector3::fromVecStr(xyz_str));
		}

		const char* rpy_str = xml->Attribute("rpy");
		if (rpy_str != NULL) {
			t.setRotation(Rotation::fromRpyStr(rpy_str));
		}
	}
	return t;
}
