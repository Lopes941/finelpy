#include <finelc/material/material.h>
#include <finelc/material/constitutive.h>

#include <finelc/binding/bindings.h>
#include <finelc/binding/matrix_binding.h>
#include <finelc/binding/material_binding.h>

#include <pybind11/pybind11.h>
#include <pybind11/eigen.h> 
#include <pybind11/numpy.h> 
#include <pybind11/stl.h>

#include <stdexcept>

using namespace finelc;
namespace py = pybind11;


void bind_material(py::module_& handle){

     {
     py::enum_<MaterialProperties>
          (handle, 
          "MaterialProperties",
          R"pbdoc(
          Represents the material properties.
          )pbdoc")

          .value("RHO", MaterialProperties::RHO, R"pbdoc(Density.)pbdoc")
          .value("YOUNGS_MOD", MaterialProperties::YOUNGS_MOD, R"pbdoc(Young's Modulus.)pbdoc")
          .value("YOUNGS_MOD_XX", MaterialProperties::YOUNGS_MOD_XX, R"pbdoc(Young's Modulus on X axis.)pbdoc")
          .value("YOUNGS_MOD_YY", MaterialProperties::YOUNGS_MOD_YY, R"pbdoc(Young's Modulus on Y axis.)pbdoc")
          .value("YOUNGS_MOD_ZZ", MaterialProperties::YOUNGS_MOD_ZZ, R"pbdoc(Young's Modulus on Z axis.)pbdoc")
          .value("POISSON", MaterialProperties::POISSON, R"pbdoc(Poisson's ratio.)pbdoc")
          .value("POISSON_XY", MaterialProperties::POISSON_XY, R"pbdoc(Poisson's ratio on XY axes.)pbdoc")
          .value("POISSON_XZ", MaterialProperties::POISSON_XZ, R"pbdoc(Poisson's ratio on XZ axes.)pbdoc")
          .value("POISSON_YZ", MaterialProperties::POISSON_YZ, R"pbdoc(Poisson's ratio on YZ axes.)pbdoc")
          .value("THERMAL_COND_X", MaterialProperties::THERMAL_COND_X, R"pbdoc(Thermal conductivity on X axis.)pbdoc")
          .value("THERMAL_COND_Y", MaterialProperties::THERMAL_COND_Y, R"pbdoc(Thermal conductivity on Y axis.)pbdoc")
          .value("THERMAL_COND_Z", MaterialProperties::THERMAL_COND_Z, R"pbdoc(Thermal conductivity on Z axis.)pbdoc")

          .value("IZZ", MaterialProperties::IZZ, R"pbdoc(Moment of inertia of beam cross section.)pbdoc")
          .value("A", MaterialProperties::A, R"pbdoc(Area of bar cross section.)pbdoc")
          ;
     }

     {
     py::enum_<ConstitutiveType>
          (handle, 
          "ConstitutiveType",
          R"pbdoc(
          Represents the constitutive models.
          )pbdoc")

          .value("PYTHON_CONSTITUTIVE", ConstitutiveType::PYTHON_CONSTITUTIVE, R"pbdoc(Python custom model)pbdoc")
          .value("BAR_LINEAR_ELASTIC", ConstitutiveType::BAR_LINEAR_ELASTIC, R"pbdoc(Linear elastic bar (Hooke's Law).)pbdoc")
          .value("BEAM_LINEAR_ELASTIC", ConstitutiveType::BEAM_LINEAR_ELASTIC, R"pbdoc(Linear elastic beam (Hooke's Law).)pbdoc")
          .value("PLANE_STRESS", ConstitutiveType::PLANE_STRESS, R"pbdoc(Linear elastic plane stress.)pbdoc")
          .value("PLANE_STRAIN", ConstitutiveType::PLANE_STRAIN, R"pbdoc(Linear elastic plane strain)pbdoc")
          .value("SOLID_LINEAR_ELASTIC", ConstitutiveType::SOLID_LINEAR_ELASTIC, R"pbdoc(Solid linear isotropic elasticity.)pbdoc")
          .value("THERMAL_CONDUCTION_1D", ConstitutiveType::THERMAL_CONDUCTION_1D, R"pbdoc(1D thermal conduction.)pbdoc")
          .value("THERMAL_CONDUCTION_2D", ConstitutiveType::THERMAL_CONDUCTION_2D, R"pbdoc(2D thermal conduction.)pbdoc")
          .value("THERMAL_CONDUCTION_3D", ConstitutiveType::THERMAL_CONDUCTION_3D, R"pbdoc(3D thermal conduction.)pbdoc")
          ;
     }

     {
     py::class_<Material, std::shared_ptr<Material>>
          (handle, 
          "Material",
          R"pbdoc(
          Material object.
          )pbdoc")

          // .def(py::init<>())

          .def_property_readonly("rho",
          [](const Material& self) -> double {
               return self.get_property(MaterialProperties::RHO);
          },
          R"pbdoc(
          Density of material.
          )pbdoc")

          .def_property_readonly("E",
          [](const Material& self) -> double {
               return self.get_property(MaterialProperties::YOUNGS_MOD);
          },
          R"pbdoc(
          Young's Modulus of material.
          )pbdoc")

          .def_property_readonly("nu",
          [](const Material& self) -> double {
               return self.get_property(MaterialProperties::POISSON);
          },
          R"pbdoc(
          Poisson's ratio of material.
          )pbdoc")

          .def_property_readonly("A",
          [](const Material& self) -> double {
               return self.get_property(MaterialProperties::A);
          },
          R"pbdoc(
          Area of cross section.
          )pbdoc")

          .def_property_readonly("IZZ",
          [](const Material& self) -> double {
               return self.get_property(MaterialProperties::IZZ);
          },
          R"pbdoc(
          Moment of inertia of cross section.
          )pbdoc")

          .def(
               "add_property", 
               [](Material& self, MaterialProperties prop, double value) -> Material& {
                    return self.add_property(prop, value);
               },
               py::arg("property"),
               py::arg("value"),
               R"pbdoc(
               Add or update material property.

               Parameters
               ------------
               property: MaterialProperties
                    Name of material property.
               value: float
                    Assingned value.
               )pbdoc")

          .def("add_property", 
               [](Material& self, py::dict prop_dict) -> Material& {
                    for(auto& prop: prop_dict){
                         MaterialProperties key = py::cast<MaterialProperties>(prop.first);
                         double value = py::cast<double>(prop.second);
                         self.add_property(key, value);
                    }
                    return self;
               },
               py::arg("prop_dict"),
               R"pbdoc(
               Add or update material property.

               Parameters
               ------------
               prop_dict: dict
                    Dictionary assigning {MaterialProperties: float}.
               )pbdoc")


          .def("get_property", 
               &Material::get_property,
               py::arg("property"),
               R"pbdoc(
               Add or update material property.

               Parameters
               ------------
               property: MaterialProperties
                    Name of material property.

               Returns
               ------------
               value: float
                    Value of this property, if it exists.
               )pbdoc")
          ;
     }


     py::class_<IConstitutiveModel, ConstitutiveModelTrampoline, IConstitutiveModel_ptr>(
          handle,
          "ConstitutiveModel",
          R"pbdoc(
          Constitutive model interface.
          )pbdoc")

          .def(py::init<>())

          .def_property_readonly("is_linear",
               &IConstitutiveModel::is_linear,
               R"pbdoc(True if constitutive model is linear.)pbdoc")

          .def_property_readonly("model",
               &IConstitutiveModel::constitutive_model,
               R"pbdoc(ID of current constitutive model.)pbdoc")

          .def("get_property",
               &IConstitutiveModel::get_property,
               py::arg("property"),
               R"pbdoc(
               Get the value of a material property

               Parameters
               ------------
               property: MaterialProperties
                    Identification of material property
               )pbdoc")

          .def("D",
               &IConstitutiveModel::D,
               py::arg("loc")=py::none(),
               py::arg("displacement")=py::none(),
               R"pbdoc(
               Get the constitutive matrix at given point.

               Parameters
               -----------
               loc: tuple (2 or 3 elements), optional (default=None)
                    Local coordinates where the value is computed.
               displacement: array, optional (default=None)
                    Nodal displacements of element, used in non-linear analyses.

               Returns
               --------
               D: numpy.array
                    Constitutive matrix evaluated at given point.
               )pbdoc")

          ;


     py::class_<MaterialCatalogue, std::shared_ptr<MaterialCatalogue>> (handle, "MaterialCatalogue")

          .def_static("get_catalogue", &MaterialCatalogue::get_catalogue,
                         "Get the singleton MaterialCatalogue")

          .def("create_material", 
               [](MaterialCatalogue& self, const std::string& name) -> Material_ptr {
                    return self.create_material(name);
               },
               py::return_value_policy::reference_internal,
               R"pbdoc(
               Create a rectangle with given dimensions and origin.

               Parameters
               ----------
               name : string
                    name of the material.
               
               Returns
               ----------
               Material
                    Reference to created material.
               )pbdoc")

          .def("get_material", &MaterialCatalogue::get_material,
               py::return_value_policy::reference_internal,
               R"pbdoc(Get an existing material by name.)pbdoc")
          ;


}