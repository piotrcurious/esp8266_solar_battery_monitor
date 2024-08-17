There are a few open-source tools that can help analyze C/C++ code and visualize function calls, dependencies, and code flow.

1. **GDB Graphs**: This tool uses `gdb` and Python to visualize function call flows for C/C++ programs. It generates graphs showing relationships between functions, which can help in understanding large codebases. The tool captures data using `gdb` and then processes it with Python to create visual representations using Graphviz and Matplotlib. It's a straightforward tool for those familiar with GDB and looking for a way to map out function calls in their projects【17†source】.

2. **Code Graph**: A Visual Studio extension that can analyze C/C++ code (among other languages) and visualize the relationships between different code elements, such as functions, classes, and variables. It integrates with Visual Studio to display these relationships interactively, allowing you to explore the structure of your codebase within the IDE. It uses Doxygen for parsing and supports a variety of graphing features, making it a versatile option for those using Visual Studio【16†source】.

3. **Doxygen with Graphviz**: While not a single tool, combining Doxygen with Graphviz allows you to generate detailed diagrams of code structure, including call graphs and dependency graphs. Doxygen parses your code to extract documentation and can generate various diagrams if configured to use Graphviz as a backend. This setup is highly customizable and widely used in the industry.

These tools can provide powerful insights into the structure and flow of your code, each with its own strengths depending on your development environment and specific needs.
