^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package fanuc_m900ib_support
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.6.0 (2025-02-15)
------------------
* remove ``--inorder`` xacro arg everywhere (`#372 <https://github.com/ros-industrial/fanuc/issues/372>`_).
* for a complete list of changes see the `commit log for 0.6.0 <https://github.com/ros-industrial/fanuc/compare/0.5.1...0.6.0>`_.

0.5.1 (2021-04-02)
------------------
* bump min CMake version (`#309 <https://github.com/ros-industrial/fanuc/issues/309>`_).
* correct missing dependency on ``industrial_robot_client`` (RViz cfg) (`#306 <https://github.com/ros-industrial/fanuc/issues/306>`_).
* point to manifest for info on package contents (`#292 <https://github.com/ros-industrial/fanuc/issues/292>`_).
* remove roslaunch version requirements as they're no longer needed (`#278 <https://github.com/ros-industrial/fanuc/issues/278>`_).
* migrate to JSP GUI (was split out of JSP).
* for a complete list of changes see the `commit log for 0.5.1 <https://github.com/ros-industrial/fanuc/compare/0.5.0...0.5.1>`_.

0.5.0 (2019-09-25)
------------------
* first release of this package.
* make ``flange`` parent of ``tool0`` (`#271 <https://github.com/ros-industrial/fanuc/issues/271>`_).
* harmonise pkg version numbers (`#260 <https://github.com/ros-industrial/fanuc/issues/260>`_).
* migrate M-900iB support pkgs from experimental repository (`#256 <https://github.com/ros-industrial/fanuc/issues/256>`_).
* update xacro xmlns uri (`#239 <https://github.com/ros-industrial/fanuc/issues/239>`_).
* add 'support level' indicators (`#232 <https://github.com/ros-industrial/fanuc/issues/232>`_).
* remove unescaped ampersand from 'robot_ip' arg doc string (`#176 <https://github.com/ros-industrial/fanuc/issues/176>`_).
* use Jade+ xacro 'pi' constant instead of our own.
* migrate to in-order processing of xacros (supported on Indigo and up).
* for a complete list of changes see the `commit log for 0.5.0 <https://github.com/ros-industrial/fanuc/compare/0.4.4...0.5.0>`_.
