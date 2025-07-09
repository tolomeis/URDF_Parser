#ifndef URDF_COMMON_H
#define URDF_COMMON_H

#include <vector>
#include <string>
#include <math.h>
#ifndef M_PI
#define M_PI 3.141592538
#endif //M_PI

#include "../tinyxml/txml.h"
#include <memory>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "exception.h"

using namespace std;

namespace urdf {

	class Vector3 : public Eigen::Vector3d {
	public:
		// Inherit constructors from Eigen::Vector3d
		using Eigen::Vector3d::Vector3d;
		
		// Default constructor
		Vector3() : Eigen::Vector3d(0., 0., 0.) {}
		
		// Copy constructor from Eigen::Vector3d
		Vector3(const Eigen::Vector3d& other) : Eigen::Vector3d(other) {}
		
		// Copy constructor
		Vector3(const Vector3& other) : Eigen::Vector3d(other) {}

		void clear() {
			x() = 0.;
			y() = 0.;
			z() = 0.;
		}

		// Provide compatibility accessors
		double& x() { return (*this)[0]; }
		double& y() { return (*this)[1]; }
		double& z() { return (*this)[2]; }
		const double& x() const { return (*this)[0]; }
		const double& y() const { return (*this)[1]; }
		const double& z() const { return (*this)[2]; }

		static Vector3 fromVecStr(const string& vector_str);
	};

	class Rotation : public Eigen::Quaterniond {
	public:
		// Inherit constructors from Eigen::Quaterniond
		using Eigen::Quaterniond::Quaterniond;
		
		// Default constructor (identity quaternion)
		Rotation() : Eigen::Quaterniond(1., 0., 0., 0.) {}
		
		// Copy constructor from Eigen::Quaterniond
		Rotation(const Eigen::Quaterniond& other) : Eigen::Quaterniond(other) {}
		
		// Copy constructor
		Rotation(const Rotation& other) : Eigen::Quaterniond(other) {}

		void clear() {
			x() = 0.;
			y() = 0.;
			z() = 0.;
			w() = 1.;
		}

		void getRpy(double &roll, double &pitch, double &yaw) const;
		void normalize() { 
			Eigen::Quaterniond::normalize(); 
		}
		Rotation getInverse() const {
			return Rotation(conjugate());
		}

		// Operators
		Rotation operator*(const Rotation& other) const {
			return Rotation(static_cast<const Eigen::Quaterniond&>(*this) * static_cast<const Eigen::Quaterniond&>(other));
		}
		
		Vector3 operator*(const Vector3& vec) const {
			return Vector3((*this) * static_cast<const Eigen::Vector3d&>(vec));
		}

		// Provide compatibility accessors  
		double& x() { return coeffs()[0]; }
		double& y() { return coeffs()[1]; }
		double& z() { return coeffs()[2]; }
		double& w() { return coeffs()[3]; }
		const double& x() const { return coeffs()[0]; }
		const double& y() const { return coeffs()[1]; }
		const double& z() const { return coeffs()[2]; }
		const double& w() const { return coeffs()[3]; }

		static Rotation fromRpy(double roll, double pitch, double yaw);
		static Rotation fromRpyStr(const string &rotation_str);
	};

	struct Color {
		float r;
		float g;
		float b;
		float a;

		void clear() {
			r = 0.;
			g = 0.;
			b = 0.;
			a = 1.;
		}

		Color() : r(0.), g(0.), b(0.), a(1.) {}
		Color(float r, float g, float b, float a) : r(r), g(g), b(b), a(a) {}
		Color(const Color& other) : r(other.r), g(other.g), b(other.b), a(other.a) {}

		static Color fromColorStr(const std::string &vector_str);
	};

	class Transform : public Eigen::Isometry3d {
	public:
		// Inherit Eigen::Isometry3d constructors
		using Eigen::Isometry3d::Isometry3d;

		// Default constructor (identity)
		Transform() : Eigen::Isometry3d(Eigen::Isometry3d::Identity()) {}

		// Copy constructor
		Transform(const Transform& other) : Eigen::Isometry3d(other) {}

		// From Eigen::Isometry3d
		Transform(const Eigen::Isometry3d& iso) : Eigen::Isometry3d(iso) {}

		void clear() {
			*this = Eigen::Isometry3d::Identity();
		}

		Vector3 position() const {
			return Vector3(this->translation());
		}

		void setPosition(const Vector3& pos) {
			this->translation() = pos;
		}

		Rotation rotation() const {
			return Rotation(Eigen::Quaterniond(this->linear()));
		}

		void setRotation(const Rotation& rot) {
			this->linear() = rot.toRotationMatrix();
		}

		// Transform multiplication for composing transforms
		Transform operator*(const Transform& other) const {
			return Transform(static_cast<const Eigen::Isometry3d&>(*this) * static_cast<const Eigen::Isometry3d&>(other));
		}

		// Conversion to Eigen::Isometry3d (no-op)
		Eigen::Isometry3d toIsometry3d() const {
			return *this;
		}

		// Create from Eigen::Isometry3d (alias)
		static Transform fromIsometry3d(const Eigen::Isometry3d& iso) {
			return Transform(iso);
		}

		static Transform fromXml(TiXmlElement* xml); // Declaration only
	};



	class Twist {
	public:
		Vector3  linear;
		Vector3  angular;

		void clear() {
			this->linear.clear();
			this->angular.clear();
		}

		Twist() : linear(Vector3()), angular(Vector3()) {}
		Twist(const Twist& other) : linear(other.linear), angular(other.angular) {}
	};
}

#endif
