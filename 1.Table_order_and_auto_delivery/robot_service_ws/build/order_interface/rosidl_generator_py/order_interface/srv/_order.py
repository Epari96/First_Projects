# generated from rosidl_generator_py/resource/_idl.py.em
# with input from order_interface:srv/Order.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_Order_Request(type):
    """Metaclass of message 'Order_Request'."""

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
            module = import_type_support('order_interface')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'order_interface.srv.Order_Request')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__order__request
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__order__request
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__order__request
            cls._TYPE_SUPPORT = module.type_support_msg__srv__order__request
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__order__request

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class Order_Request(metaclass=Metaclass_Order_Request):
    """Message class 'Order_Request'."""

    __slots__ = [
        '_item_name',
        '_quantity',
        '_is_receipt',
    ]

    _fields_and_field_types = {
        'item_name': 'string',
        'quantity': 'int32',
        'is_receipt': 'int32',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.item_name = kwargs.get('item_name', str())
        self.quantity = kwargs.get('quantity', int())
        self.is_receipt = kwargs.get('is_receipt', int())

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
        if self.item_name != other.item_name:
            return False
        if self.quantity != other.quantity:
            return False
        if self.is_receipt != other.is_receipt:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def item_name(self):
        """Message field 'item_name'."""
        return self._item_name

    @item_name.setter
    def item_name(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'item_name' field must be of type 'str'"
        self._item_name = value

    @builtins.property
    def quantity(self):
        """Message field 'quantity'."""
        return self._quantity

    @quantity.setter
    def quantity(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'quantity' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'quantity' field must be an integer in [-2147483648, 2147483647]"
        self._quantity = value

    @builtins.property
    def is_receipt(self):
        """Message field 'is_receipt'."""
        return self._is_receipt

    @is_receipt.setter
    def is_receipt(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'is_receipt' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'is_receipt' field must be an integer in [-2147483648, 2147483647]"
        self._is_receipt = value


# Import statements for member types

# already imported above
# import builtins

# already imported above
# import rosidl_parser.definition


class Metaclass_Order_Response(type):
    """Metaclass of message 'Order_Response'."""

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
            module = import_type_support('order_interface')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'order_interface.srv.Order_Response')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__order__response
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__order__response
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__order__response
            cls._TYPE_SUPPORT = module.type_support_msg__srv__order__response
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__order__response

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class Order_Response(metaclass=Metaclass_Order_Response):
    """Message class 'Order_Response'."""

    __slots__ = [
        '_success',
        '_message',
    ]

    _fields_and_field_types = {
        'success': 'boolean',
        'message': 'string',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.success = kwargs.get('success', bool())
        self.message = kwargs.get('message', str())

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
        if self.success != other.success:
            return False
        if self.message != other.message:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def success(self):
        """Message field 'success'."""
        return self._success

    @success.setter
    def success(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'success' field must be of type 'bool'"
        self._success = value

    @builtins.property
    def message(self):
        """Message field 'message'."""
        return self._message

    @message.setter
    def message(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'message' field must be of type 'str'"
        self._message = value


class Metaclass_Order(type):
    """Metaclass of service 'Order'."""

    _TYPE_SUPPORT = None

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('order_interface')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'order_interface.srv.Order')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._TYPE_SUPPORT = module.type_support_srv__srv__order

            from order_interface.srv import _order
            if _order.Metaclass_Order_Request._TYPE_SUPPORT is None:
                _order.Metaclass_Order_Request.__import_type_support__()
            if _order.Metaclass_Order_Response._TYPE_SUPPORT is None:
                _order.Metaclass_Order_Response.__import_type_support__()


class Order(metaclass=Metaclass_Order):
    from order_interface.srv._order import Order_Request as Request
    from order_interface.srv._order import Order_Response as Response

    def __init__(self):
        raise NotImplementedError('Service classes can not be instantiated')
