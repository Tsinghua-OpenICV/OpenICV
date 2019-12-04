# Configuration Files
Json Example:
```json
{
    // Shorter version
    "<Node Name>": {
        "type": "<Node type>",
        "function": "<Function type>",
        "input": "<Input node name at first port>", // Optional
        "output": "<Name of first output port>" // Optional
    },

    // Full version
    "<Node Name>": {
        "type": "<Node type>",
        "function": {
            "type": "<Function type>",
            "<Function param name>": "<Param value>",
            "<Function param name>": "<Param value>",
            ...
        },
        "input": [
            "<Input node name at First Port>",
            {
                "name": "[Name of second input port]", // Optional
                "connect": [ // Optional
                    "<Input node name at second Port>",
                    "<Input node Name at second Port>@<Output port index of input node>",
                    "<Input node Name at second Port>@<Output port name of Input Node>",
                    ...
                ],
                "<Input port param name>": "<Param value>",
                ...
            },
            ...
        ],
        "output": [
            "<Name of first output port>",
            {
                "name": "[Name of second output port]", # Optional
                "connect": [
                    "<Output node name at second port>",
                    "<Output node name at second port>@<Input port index of output node>",
                    "<Output node name at second port>@<Input port name of output node>",
                    ...
                ],
                "<Output port param name>": "<Param value>",
                ...
            },
            ...
        ],
        "<Node param name>": "<Param value>",
        ...
    },

    ...
}
```

INI Example: (**To be supported**)
```ini
; Shorter Version
[<Node Name>]
type=<Node type>
function=<Function type>
input=<Input node name at first port> ; Optional
output=<Name of first output port> ; Optional

; Full Version
[<Node Name>]
type=<Node type>
<Node param name>=<Param value>
<Node param name>=<Param value>
...

[<Node Name>.function]
type=<Function Type>
<Function param name>=<Param value>
<Function param name>=<Param value>
...

[<Node Name>.input]
connect=<Input node name at First Port>

[<Node Name>.input]
name=[Name of second input port] ; Optional
connect=<Input node name at second Port>
connect=<Input node name at second Port>@<Output port index of input node>
connect=<Input node name at second Port>@<Output port name of Input Node>
<Input port param name>=<Param value>
...

[<Node Name>.output]
name=<Name of first output port>

[<Node Name>.output@1]
name=[Name of second output port]
connect=<Output node name at second port>
connect=<Output node name at second port>@<Input port index of output node>
connect=<Output node name at second port>@<Input port name of output node>
<Output port param name>=<Param value>
...

...

```

XML Example: (**To be supported**)
```xml
<!-- Shorter version -->
<Node_Name type="<Node type>"
           function="<Function Type>"
           input="<Input node name at first port>"/>

<!-- Full version -->
<Node_Name type="<Node type>"
           Node_param_name="<Param value>"
           ...>
    <function type="<Function type>"
              Function_Param_Name="<Param value>"
              Function_Param_Name="<Param value>"
              ...
              />
    <input connect="<Input node name at First Port>"/>
    <input name="[Name of second input port]"
            Input_port_param_name="Param value"
            ...>
        <connect>Input node name at second Port</connect>
        <connect>Input node Name at second Port@Output port index of input node</connect>
        <connect>Input node Name at second Port@Output port name of Input Node</connect>
        ...
    </input>
    ...
    <output name="<Name of first output port>"/>
    <output name="[Name of second output port]"
            Output_port_param_name="<Param value>"
            ...>
        <connect>Output node name at second port</connect>
        <connect>Output node name at second port@Input port index of output node</connect>
        <connect>Output node name at second port@Input port name of output node</connect>
        ...
    </output>
    ...
</Node_Name>

...
```

YAML Example (**To be supported**):
```yaml
# Shorter version
<Node Name>:
    type: '<Node type>'
    function: '<Function type>'
    input: '<Input node name at first port>' # Optional
    output: '<Name of first output port>' # Optional

# Full version
<Node Name>:
    type: '<Node type>'
    function:
        type: '<Function type>'
        <Function param name>: <Param value>
        <Function param name>: <Param value>
        ...
    input:
        - '<Input node name at First Port>'
        -
            name: '[Name of second input port]' # Optional
            connect: # Optional
                - '<Input node name at second Port>'
                - '<Input node Name at second Port>@<Output port index of input node>'
                - '<Input node Name at second Port>@<Output port name of Input Node>'
                - ...
            <Input port param name>: <Param value>
            ...
        - ...
    output:
        - '<Name of first output port>'
        - 
            name: '[Name of second output port]' # Optional
            connect:
                - '<Output node name at second port>'
                - '<Output node name at second port>@<Input port index of output node>'
                - '<Output node name at second port>@<Input port name of output node>'
                - ...
            <Output port param name>: <Param value>
            ...
        - ...
    <Node param name>: <Param value>
    ...

...
```

> Note:
> - By default, the node will be connected at the first port if only node name is specified in `connect` field.
> - Connnections can be defined in either input port or output port. However, if same connection is defined between same input and output, there will be warnings although this is a legal situation.
