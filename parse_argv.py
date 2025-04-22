from dataclasses import is_dataclass, fields, Field, MISSING
from typing import Any, Callable, TypeVar
import sys
import builtins


def get_global_object(obj: type | str, source_scope_globals: dict[str] = None) -> type:
    """
    Due to a "from __future__ import annotations" import, annotated types may be converted to strings.
    This function tries to interpret this string safely back into a type.
    """
    if source_scope_globals is None:
        source_scope_globals = globals()

    if isinstance(obj, type):
        return obj
    if not isinstance(obj, str):
        raise ValueError(f"Unable to interpret variable '{obj}' into a type")
    try:
        return vars(builtins)[obj]
    except KeyError as e:
        try:
            return source_scope_globals[obj]
        except KeyError as e2:
            raise KeyError(f"Unknown global var {obj}") from e2


class ArgvParser:
    EQ_SEP = "="
    USAGE = "Usage: --[arg name]=[arg value]"

    def __init__(self, dataclass_object: type, source_scope_globals: dict[str] = None):
        if source_scope_globals is None:
            source_scope_globals = globals()
        self._source_scope_globals = source_scope_globals
        dataclass_object = get_global_object(dataclass_object, self._source_scope_globals)
        if not is_dataclass(dataclass_object):
            raise ValueError("Argument type must be a dataclass Object")
        self._dataclass_object = dataclass_object
        self._dataclass_fields = fields(dataclass_object)
        self._arg_dict: dict[str] = {}
    
    def _available_args(self) -> list[str]:
        return [field.name for field in self._dataclass_fields]
    
    @classmethod
    def _field_cli_display(cls, field: Field) -> str:
        return "--" + field.name + cls.EQ_SEP
    
    def _match_field(self, cli_arg: str, field: Field) -> bool:
        return cli_arg.startswith(ArgvParser._field_cli_display(field))
    
    def _eval_str_field_val(self, val: str, field: Field) -> Any:
        field_type = get_global_object(field.type, self._source_scope_globals)
        if field_type in (int, float, str):
            return field_type(val)
        if field_type is bool:
            val_ = val.lower()
            try:
                return {"true": True, "false": False}[val_]
            except KeyError as e:
                raise ValueError(f"Cannot interpret '{val}' as bool") from e
        print(f"Warning: don't know how to evaluate field type {field_type}")
        return field_type(val)
            
    def _parse_field(self, cli_arg: str, field: Field) -> Any:
        if not self._match_field(cli_arg, field):
            raise ValueError("Field doesn't match")
        str_val = cli_arg[cli_arg.index(ArgvParser.EQ_SEP)+1:]
        return self._eval_str_field_val(str_val, field)
    
    def _parse_arg(self, cli_arg: str) -> None:
        for field in self._dataclass_fields:
            if self._match_field(cli_arg, field):
                self._arg_dict[field.name] = self._parse_field(cli_arg, field)
                return
        raise ValueError(f"Unknown cli argument or bad synthax: {cli_arg}\n{ArgvParser.USAGE}\nAvailable arguments: {', '.join(self._available_args())}")

    def _enforce_required_fields(self) -> None:
        for field in self._dataclass_fields:
            if field.default is MISSING:
                if field.name not in self._arg_dict:
                    raise ValueError(f"Didn't provide a value for required argument '{field.name}'")

    def parse_argv(self) -> Any:
        self._arg_dict.clear()
        for arg in sys.argv[1:]:
            self._parse_arg(arg)
        self._enforce_required_fields()
        return self._dataclass_object(**self._arg_dict)


T = TypeVar('T')


def parse_argv(f: Callable[[Any], T]) -> Callable[[], T]:
    """
    Assuming the function f takes exactly one argument which must be annotated as a dataclass instance
    """
    annots = {k: v for k, v in f.__annotations__.items() if k != "return"}
    if len(annots) != 1:
        raise ValueError(f"The function {f.__name__} must take exactly one argument which must be annotated as a dataclass instance")
    dataclass_object = list(annots.values())[0]
    parser = ArgvParser(dataclass_object, source_scope_globals=f.__globals__)
    parsed_arg = parser.parse_argv()

    def inner_f() -> T:
        return f(parsed_arg)
    
    return inner_f
