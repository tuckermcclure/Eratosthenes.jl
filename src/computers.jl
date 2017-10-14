mutable struct Computer
    name::String
    board::DynamicalModel
    software::Vector{DynamicalModel}
    Computer(name = "computer",
             board = DynamicalModel("null_board"),
             software = Vector{DynamicalModel}()) = new(name, board, software)
end
