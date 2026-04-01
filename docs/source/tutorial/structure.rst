FinelPy Structure
===================

This library is organized in multiple modules. Each one responsible for each part of the FEM algorithm.

Geometry Module
----------------

Responsible for holding geometrical features. The main objects here use the :class:`IGeometry<finelpy.geometry.IGeometry>` interface.

It provides a method for checking if a point is inside it, and methods for getting the nodes and number of nodes associated with it, provided a mesh has been already created with it.

.. code-block:: python

    import finelpy as fp

    dimensions = (10,5)
    origin = (0,0,0)

    rectangle = fp.geometry.Rectangle(dimensions, origin)

    p1 = (5,2)
    print(rectangle.is_inside(p1))
    # >> True

    p2 = (15,7)
    print(rectangle.is_inside(p2))
    # >> False



Material Module
----------------

Responsible for holding the material and the constitutive models. The main objects here are the :class:`Material <finelpy.material.Material>` object and the :class:`Constitutive Model <finelpy.material.ConstitutiveModel>` interface.

Custom ConstitutiveModel objects can be defined in pure-python language, see :ref:`reference-constitutive_page` for more information.

A quick example with a custom Constitutive model and with a custom material is shown below:

.. code-block:: python

    import numpy as np
    import finelpy as fp
    import finelpy.material as mat

    class BarModel(mat.pyConstitutiveModel):
        
        def __init__(self, material):
            super().__init__(material)
            
        def D(self, loc=None, disp=None):
            E = self.get_property(mat.MaterialProperties.YOUNGS_MOD)
            return np.array([[E]])

    properties = {
        mat.MaterialProperties.YOUNGS_MOD: 210e9,
        mat.MaterialProperties.RHO: 7850,
        mat.MaterialProperties.POISSON: 0.3
    }
    steel = mat.create_material("steel").add_property(properties)

    model = BarModel(steel)
    print(model.D())
    # >> [[2.1e+11]]



Element Module
---------------

Responsible for holding information over the elements, and calculating everything regarding elements. The main object here is the :class:`Element <finelpy.element.Element>` interface.

In the FinelPy framework, elements are defined by three models: Geometry, Physics and Constitutive.

* **Geometry**: describes the objects geometical features; i.e. number of nodes, dimension, shape functions (for the mapping), Jacobian matrix etc

* **Physics**: responsible for the kinematic model and everything related to it. Responsible for: B matrix, interpolation functions (with bubble functions), reconstructing strain, stress, heat flux, etc.

* **Constitutive**: responsible for the material model in the element. Responsible for the D matrix.

If any of those are defined as non-linear, then the element is non-linear. Equal linear elements use the same matrix flyweight, thus, their stiffness and mass matrices are calculated only once.

Elements can be defined using these standard models provided by the library, using custom standard models or by direct definition of the element. The exact procedure is shown in :ref:`reference-element_page`.



.. code-block:: python

    import numpy as np
    import finelpy as fp
    import finelpy.material as mat
    import finelpy.element as elem


    properties = {
        mat.MaterialProperties.YOUNGS_MOD: 210e9,
        mat.MaterialProperties.RHO: 7850,
        mat.MaterialProperties.POISSON: 0.3
    }
    steel = mat.create_material("steel").add_property(properties)

    el = elem.create_element(
            elem.ShapeType.QUAD4, 
            elem.ModelType.PLANE_STRUCTURAL,
            mat.ConstitutiveType.PLANE_STRESS, 
            steel)

    print(el.N((0,0)))
    # >> [[0.25 0.   0.25 0.   0.25 0.   0.25 0.  ]
    #     [0.   0.25 0.   0.25 0.   0.25 0.   0.25]]


.. warning:: 

    Certain functions require initialized elements. Elements are only initialized when they are part of a mesh.



Mesh Module
------------

Responsible for holding information over the mesh. The main object is the :class:`Mesh <finelpy.mesh.Mesh>` interface.

Must be created by a :class:`MeshBuilder <finelpy.mesh.MeshBuilder>`. Examples of mesh builders are seen at :ref:`reference_mesh_builder`.

To build a mesh, usually a :class:`Geometry <finelpy.geometry.IGeometry>` and a base :class:`Element <finelpy.element.Element>` must be provided.

.. code-block:: python

    import numpy as np
    import finelpy as fp
    import finelpy.geometry as geo
    import finelpy.material as mat
    import finelpy.element as elem
    import finelpy.mesh as msh

    dimensions = (10,5)
    origin = (0,0,0)

    rectangle = geo.Rectangle(dimensions, origin)

    properties = {
        mat.MaterialProperties.YOUNGS_MOD: 210e9,
        mat.MaterialProperties.RHO: 7850,
        mat.MaterialProperties.POISSON: 0.3
    }
    steel = mat.create_material("steel").add_property(properties)

    el = elem.create_element(
            elem.ShapeType.QUAD4, 
            elem.ModelType.PLANE_STRUCTURAL,
            mat.ConstitutiveType.PLANE_STRESS, 
            steel)

    nx = 2
    ny = 2
    mesh_gen = msh.RectangularMesh(rectangle, el)
    mesh_gen.set_grid(nx,ny)
    rec_mesh = mesh_gen.build()

    print(rec_mesh.centers)
    # >> [[2.5  1.25 0.  ]
    #     [7.5  1.25 0.  ]
    #     [2.5  3.75 0.  ]
    #     [7.5  3.75 0.  ]]


Analysis Module
----------------

Responsible for applying boundary conditions, updating the mesh with following an interpolation scheme. The main object here is the :class:`Analysis <finelpy.analysis.Analysis>` object.

Similar to a mesh object, it is created with an :class:`Analysis Builder <finelpy.analysis.AnalysisBuilder>` class.

An example, with applications of boundary conditions is shown below. More information over it can be found at :ref:`reference_boundary_condition`.

.. note::

    You can specific methods from :class:`Geometry <finelpy.geometry.IGeometry>` objects to get the nodes of application.
    
    You can also define new geometries. The nodes are detected automatically by the library internal functions.



.. code-block:: python

    import numpy as np
    import finelpy as fp
    import finelpy.geometry as geo
    import finelpy.material as mat
    import finelpy.element as elem
    import finelpy.mesh as msh
    import finelpy.analysis as analysis

    dimensions = (10,5)
    origin = (0,0,0)

    rectangle = geo.Rectangle(dimensions, origin)

    properties = {
        mat.MaterialProperties.YOUNGS_MOD: 210e9,
        mat.MaterialProperties.RHO: 7850,
        mat.MaterialProperties.POISSON: 0.3
    }
    steel = mat.create_material("steel").add_property(properties)

    el = elem.create_element(
            elem.ShapeType.QUAD4, 
            elem.ModelType.PLANE_STRUCTURAL,
            mat.ConstitutiveType.PLANE_STRESS, 
            steel)

    nx = 2
    ny = 2
    mesh_gen = msh.RectangularMesh(rectangle, el)
    mesh_gen.set_grid(nx,ny)
    rec_mesh = mesh_gen.build()


    forc_node = rec_mesh.find_node((80,25)) #Location of force

    an_gen = analysis.AnalysisBuilder(rec_mesh)
    an_gen.add_boundary_condition(analysis.DOFType.UX,rectangle.left_side,0)
    an_gen.add_boundary_condition(analysis.DOFType.UY,rectangle.left_side,0)
    an_gen.add_force(analysis.DOFType.UY,forc_node,-10)

    an = an_gen.build()
    print(an.free_dofs)
    # >> [ 2  3  4  5  8  9 10 11 14 15 16 17]


Solver and Result Modules
---------------------------

The modules for solving and analysing the results are so intrinsically connected that it's better to discuss them together.

The Solver module contains the solver classes, each for a different type of analysis, such as static equilibrium, transient analysis, eigenproblems, etc. 

The Result module contains the object that holds the solution for each type of solver problem. It contains the methods for plotting and printing the results.

.. code-block:: python

    import numpy as np
    import finelpy as fp
    import finelpy.geometry as geo
    import finelpy.material as mat
    import finelpy.element as elem
    import finelpy.mesh as msh
    import finelpy.analysis as analysis
    import finelpy.solver as solver

    dimensions = (10,5)
    origin = (0,0,0)

    rectangle = geo.Rectangle(dimensions, origin)

    properties = {
        mat.MaterialProperties.YOUNGS_MOD: 210e9,
        mat.MaterialProperties.RHO: 7850,
        mat.MaterialProperties.POISSON: 0.3
    }
    steel = mat.create_material("steel").add_property(properties)

    el = elem.create_element(
            elem.ShapeType.QUAD4, 
            elem.ModelType.PLANE_STRUCTURAL,
            mat.ConstitutiveType.PLANE_STRESS, 
            steel)

    nx = 2
    ny = 2
    mesh_gen = msh.RectangularMesh(rectangle, el)
    mesh_gen.set_grid(nx,ny)
    rec_mesh = mesh_gen.build()


    forc_node = rec_mesh.find_node((80,25)) #Location of force

    an_gen = analysis.AnalysisBuilder(rec_mesh)
    an_gen.add_boundary_condition(analysis.DOFType.UX,rectangle.left_side,0)
    an_gen.add_boundary_condition(analysis.DOFType.UY,rectangle.left_side,0)
    an_gen.add_force(analysis.DOFType.UY,forc_node,-10)

    an = an_gen.build()

    sol = solver.StaticSolver(an)
    res = sol.solve()

    print(res.u)
    # >> [ 0.00000000e+00  0.00000000e+00 -2.94823199e-10 -4.34347411e-10
    #      -3.90399285e-10 -1.21405487e-09  0.00000000e+00  0.00000000e+00
    #      -4.11878279e-12 -4.09762106e-10  3.89663224e-13 -1.23080127e-09
    #       0.00000000e+00  0.00000000e+00  2.94699805e-10 -4.20412480e-10
    #       4.16546303e-10 -1.28680197e-09]