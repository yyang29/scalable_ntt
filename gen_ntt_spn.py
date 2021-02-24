# Generate SPN

def gen_spn(out_file, spn_id, routing, io_width, dp):

    print('generating SPN %d with routing stride %s, io_width %d, dp %d...'
          % (spn_id, routing, io_width, dp))

    switch_sv = f"""// This verilog file is generated automatically.
// Author: Yang Yang (yyang172@usc.edu)

module switch (
    inData_0,
    inData_1,
    outData_0,
    outData_1,
    ctrl,
    clk,
    rst
  );

  input ctrl, clk, rst;
  input         [{io_width}-1:0] inData_0, inData_1;
  output logic  [{io_width}-1:0] outData_0, outData_1;

  always_ff @ (posedge clk) begin
    if (rst) begin
      outData_0 <= 0;
      outData_1 <= 0;
    end else begin
      outData_0 <= (!ctrl) ? inData_0 : inData_1;
      outData_1 <= (!ctrl) ? inData_1 : inData_0;
    end
  end

endmodule

"""
    with open(out_file, 'a+') as fid:
        fid.write(switch_sv)
