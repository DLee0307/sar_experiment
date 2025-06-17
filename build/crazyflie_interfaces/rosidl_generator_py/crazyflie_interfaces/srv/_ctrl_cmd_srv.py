# generated from rosidl_generator_py/resource/_idl.py.em
# with input from crazyflie_interfaces:srv/CTRLCmdSrv.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_CTRLCmdSrv_Request(type):
    """Metaclass of message 'CTRLCmdSrv_Request'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
    }

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('crazyflie_interfaces')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'crazyflie_interfaces.srv.CTRLCmdSrv_Request')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__ctrl_cmd_srv__request
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__ctrl_cmd_srv__request
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__ctrl_cmd_srv__request
            cls._TYPE_SUPPORT = module.type_support_msg__srv__ctrl_cmd_srv__request
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__ctrl_cmd_srv__request

            from geometry_msgs.msg import Vector3
            if Vector3.__class__._TYPE_SUPPORT is None:
                Vector3.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class CTRLCmdSrv_Request(metaclass=Metaclass_CTRLCmdSrv_Request):
    """Message class 'CTRLCmdSrv_Request'."""

    __slots__ = [
        '_cmd_type',
        '_cmd_vals',
        '_cmd_flag',
        '_cmd_rx',
    ]

    _fields_and_field_types = {
        'cmd_type': 'uint16',
        'cmd_vals': 'geometry_msgs/Vector3',
        'cmd_flag': 'uint16',
        'cmd_rx': 'boolean',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('uint16'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['geometry_msgs', 'msg'], 'Vector3'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint16'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.cmd_type = kwargs.get('cmd_type', int())
        from geometry_msgs.msg import Vector3
        self.cmd_vals = kwargs.get('cmd_vals', Vector3())
        self.cmd_flag = kwargs.get('cmd_flag', int())
        self.cmd_rx = kwargs.get('cmd_rx', bool())

    def __repr__(self):
        typename = self.__class__.__module__.split('.')
        typename.pop()
        typename.append(self.__class__.__name__)
        args = []
        for s, t in zip(self.__slots__, self.SLOT_TYPES):
            field = getattr(self, s)
            fieldstr = repr(field)
            # We use Python array type for fields that can be directly stored
            # in them, and "normal" sequences for everything else.  If it is
            # a type that we store in an array, strip off the 'array' portion.
            if (
                isinstance(t, rosidl_parser.definition.AbstractSequence) and
                isinstance(t.value_type, rosidl_parser.definition.BasicType) and
                t.value_type.typename in ['float', 'double', 'int8', 'uint8', 'int16', 'uint16', 'int32', 'uint32', 'int64', 'uint64']
            ):
                if len(field) == 0:
                    fieldstr = '[]'
                else:
                    assert fieldstr.startswith('array(')
                    prefix = "array('X', "
                    suffix = ')'
                    fieldstr = fieldstr[len(prefix):-len(suffix)]
            args.append(s[1:] + '=' + fieldstr)
        return '%s(%s)' % ('.'.join(typename), ', '.join(args))

    def __eq__(self, other):
        if not isinstance(other, self.__class__):
            return False
        if self.cmd_type != other.cmd_type:
            return False
        if self.cmd_vals != other.cmd_vals:
            return False
        if self.cmd_flag != other.cmd_flag:
            return False
        if self.cmd_rx != other.cmd_rx:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def cmd_type(self):
        """Message field 'cmd_type'."""
        return self._cmd_type

    @cmd_type.setter
    def cmd_type(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'cmd_type' field must be of type 'int'"
            assert value >= 0 and value < 65536, \
                "The 'cmd_type' field must be an unsigned integer in [0, 65535]"
        self._cmd_type = value

    @builtins.property
    def cmd_vals(self):
        """Message field 'cmd_vals'."""
        return self._cmd_vals

    @cmd_vals.setter
    def cmd_vals(self, value):
        if __debug__:
            from geometry_msgs.msg import Vector3
            assert \
                isinstance(value, Vector3), \
                "The 'cmd_vals' field must be a sub message of type 'Vector3'"
        self._cmd_vals = value

    @builtins.property
    def cmd_flag(self):
        """Message field 'cmd_flag'."""
        return self._cmd_flag

    @cmd_flag.setter
    def cmd_flag(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'cmd_flag' field must be of type 'int'"
            assert value >= 0 and value < 65536, \
                "The 'cmd_flag' field must be an unsigned integer in [0, 65535]"
        self._cmd_flag = value

    @builtins.property
    def cmd_rx(self):
        """Message field 'cmd_rx'."""
        return self._cmd_rx

    @cmd_rx.setter
    def cmd_rx(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'cmd_rx' field must be of type 'bool'"
        self._cmd_rx = value


# Import statements for member types

# already imported above
# import builtins

# already imported above
# import rosidl_parser.definition


class Metaclass_CTRLCmdSrv_Response(type):
    """Metaclass of message 'CTRLCmdSrv_Response'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
    }

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('crazyflie_interfaces')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'crazyflie_interfaces.srv.CTRLCmdSrv_Response')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__ctrl_cmd_srv__response
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__ctrl_cmd_srv__response
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__ctrl_cmd_srv__response
            cls._TYPE_SUPPORT = module.type_support_msg__srv__ctrl_cmd_srv__response
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__ctrl_cmd_srv__response

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class CTRLCmdSrv_Response(metaclass=Metaclass_CTRLCmdSrv_Response):
    """Message class 'CTRLCmdSrv_Response'."""

    __slots__ = [
        '_srv_success',
    ]

    _fields_and_field_types = {
        'srv_success': 'boolean',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.srv_success = kwargs.get('srv_success', bool())

    def __repr__(self):
        typename = self.__class__.__module__.split('.')
        typename.pop()
        typename.append(self.__class__.__name__)
        args = []
        for s, t in zip(self.__slots__, self.SLOT_TYPES):
            field = getattr(self, s)
            fieldstr = repr(field)
            # We use Python array type for fields that can be directly stored
            # in them, and "normal" sequences for everything else.  If it is
            # a type that we store in an array, strip off the 'array' portion.
            if (
                isinstance(t, rosidl_parser.definition.AbstractSequence) and
                isinstance(t.value_type, rosidl_parser.definition.BasicType) and
                t.value_type.typename in ['float', 'double', 'int8', 'uint8', 'int16', 'uint16', 'int32', 'uint32', 'int64', 'uint64']
            ):
                if len(field) == 0:
                    fieldstr = '[]'
                else:
                    assert fieldstr.startswith('array(')
                    prefix = "array('X', "
                    suffix = ')'
                    fieldstr = fieldstr[len(prefix):-len(suffix)]
            args.append(s[1:] + '=' + fieldstr)
        return '%s(%s)' % ('.'.join(typename), ', '.join(args))

    def __eq__(self, other):
        if not isinstance(other, self.__class__):
            return False
        if self.srv_success != other.srv_success:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def srv_success(self):
        """Message field 'srv_success'."""
        return self._srv_success

    @srv_success.setter
    def srv_success(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'srv_success' field must be of type 'bool'"
        self._srv_success = value


class Metaclass_CTRLCmdSrv(type):
    """Metaclass of service 'CTRLCmdSrv'."""

    _TYPE_SUPPORT = None

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('crazyflie_interfaces')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'crazyflie_interfaces.srv.CTRLCmdSrv')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._TYPE_SUPPORT = module.type_support_srv__srv__ctrl_cmd_srv

            from crazyflie_interfaces.srv import _ctrl_cmd_srv
            if _ctrl_cmd_srv.Metaclass_CTRLCmdSrv_Request._TYPE_SUPPORT is None:
                _ctrl_cmd_srv.Metaclass_CTRLCmdSrv_Request.__import_type_support__()
            if _ctrl_cmd_srv.Metaclass_CTRLCmdSrv_Response._TYPE_SUPPORT is None:
                _ctrl_cmd_srv.Metaclass_CTRLCmdSrv_Response.__import_type_support__()


class CTRLCmdSrv(metaclass=Metaclass_CTRLCmdSrv):
    from crazyflie_interfaces.srv._ctrl_cmd_srv import CTRLCmdSrv_Request as Request
    from crazyflie_interfaces.srv._ctrl_cmd_srv import CTRLCmdSrv_Response as Response

    def __init__(self):
        raise NotImplementedError('Service classes can not be instantiated')
