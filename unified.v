parameter TAGMSB = 31;
parameter TAGLSB = 14;

//modulo de memoria de dados para captura dos dados a partir da chave
module dm_cache_data(
	input clk,				//clock
	input data_req_we,			//cache memory request write enable
	input [9:0] data_req_index,		//cache memory request indice 
	input [127:0] data_write,		//dado de escrita
	output [127:0] data_read		//dado de leitura
);
	reg [127:0] data_mem [0:1023]; 		//memoria de dados de 1024 entradas de 128 bits
	initial  begin
		for (int i=0; i<1024; i++) 	//setando valores iniciais de data_mem como 0
			data_mem[i] = 0;
	end
	assign data_read = data_mem[data_req_index];
	always @(posedge clk) begin
		if  (data_req_we)   
			data_mem[data_req_index] <= data_write;
	end
endmodule
//modulo da memoria de tags para mapeamento das chaves
module dm_cache_tag(
	input clk,
	input tag_req_we,			//write enable da memoria de tags
	input [9:0]tag_req_index,		//indice da memoria de tags
	input tag_write_valid,			//campo valido da memoria de tags de entrada(memoria de escrita)
	input tag_write_dirty,			//campo dirty da memoria de tags de entrada(memoria de escrita)
	input [17:0] tag_write_tag,		//campo tag da memoria de tags de entrada(memoria de escrita)
	output tag_read_valid,			//campo valido da memoria de tags de saida(memoria de leitura)
	output tag_read_dirty,                  //campo dirty da memoria de tags de saida(memoria de leitura)
	output [17:0] tag_read_tag              //campo tag da memoria de tags de saida(memoria de leitura)
);
	reg tag_mem_valid[0:1023];
	reg tag_mem_dirty[0:1023];
        reg [17:0] tag_mem_tag[0:1023];		//1024 entradas de 18 bits
	initial  begin
		for (int i=0; i<1024; i++) begin
			tag_mem_valid[i] = 0; 	//inicializando tag_mem_valid com 0 
			tag_mem_dirty[i] = 0; 	//inicializando tag_mem_dirty com 0 
			tag_mem_tag[i] = 0; 	//inicializando tag_mem_tag com 0 
		end
			
	end
	assign tag_read_dirty = tag_mem_dirty[tag_req_index];
	assign tag_read_tag = tag_mem_tag[tag_req_index];
	assign tag_read_valid = tag_mem_valid[tag_req_index];
	always @(posedge clk) begin
		if  (tag_req_we) begin
			tag_mem_tag[tag_req_index] <= tag_write_tag;		//tag_mem_tag com indice recebe tag_write_tag
			tag_mem_dirty[tag_req_index] <= tag_write_dirty;	//tag_mem_dirty com indice recebe tag_write_dirty
			tag_mem_valid[tag_req_index] <= tag_write_valid;	//tag_mem_valid com indice recebe tag_write_valid
		end
	end
endmodule
module dm_cache_fsm(
	input  clk, 					//clock
	input  rst,					//reset flag
	input cpu_req_valid,				//campo valido do CPU REQUEST 
	input cpu_req_rw,                               //campo
	input [31:0] cpu_req_addr,			//
	input [31:0] cpu_req_data,                      //
	input mem_data_ready,                           //
	input [127:0] mem_data_data,                    //
	output mem_req_valid,				//
	output mem_req_rw,                              //
	output [31:0] mem_req_addr,                     //
	output [127:0] mem_req_data,                    //
	output cpu_res_ready,				//
	output [31:0] cpu_res_data                      //
);                                                      
	parameter cache_state_idle = 2'b00;		//cache_state_idle recebe 0	
	parameter cache_state_compare_tag = 2'b01;	//cache_state_compare_tag recebe 1
	parameter cache_state_allocate = 2'b10;		//cache_state_allocate recebe 2
	parameter cache_state_write_back = 2'b11;	//cache_state_write_back recebe 3
	reg [1:0] vstate,  rstate;			//
	wire tag_read_valid;                            //
	wire tag_read_dirty;                            //
	wire [17:0] tag_read_tag;                       //
	reg tag_write_valid;                            //
	reg tag_write_dirty;                            //
	reg [17:0] tag_write_tag;                       //
	reg tag_req_we;                                 //
	reg [9:0] tag_req_index;                        //
	wire [127:0] data_read;                         //
	reg [127:0] data_read;                          //
	reg data_req_we;                                //
	reg [9:0] data_req_index;                       //
	reg v_cpu_res_ready;                            //
	reg [31:0] v_cpu_res_data;			//
	reg v_mem_req_valid;                            //
	reg v_mem_req_rw;                               //
	reg [31:0] v_mem_req_addr;                      //
	reg [127:0] v_mem_req_data;                     //
	assign mem_req_rw = v_mem_req_rw;               //conecta mem_req_rw a saida v_mem_req_rw
	assign mem_req_addr = v_mem_req_addr;           //conecta mem_req_addr a saida v_mem_req_addr
	assign mem_req_data = v_mem_req_data;           //conecta mem_req_data a saida v_mem_req_data
	assign mem_req_valid = v_mem_req_valid;         //conecta mem_req_valid a saida v_mem_req_valid
	assign cpu_res_data = v_cpu_res_data;           //conecta cpu_res_data a saida v_cpu_res_data
	assign cpu_res_ready = v_cpu_res_ready;         //conecta cpu_res_ready a saida v_cpu_res_ready
	always @* begin
		vstate = rstate; 			// vstate recebe rstate
		v_cpu_res_data = 0; 			//v_cpu_res_data recebe 0
		v_cpu_res_ready = 0;			//v_cpu_res_ready recebe 0
		tag_write_tag = 0;			//tag_write_tag recebe 0
		tag_write_dirty = 0;			//tag_write_dirty recebe 0
		tag_write_valid = 0;			//tag_write_valid recebe 0
		tag_req_we = 0;				//tag_req_we recebe 0
		tag_req_index = cpu_req_addr[13:4]; 	//tag_req_index recebe cpu_req_addr dos bits de 4 a 13 
		data_req_we = 0;			//data_req_we recebe 0
		data_req_index = cpu_req_addr[13:4];	//data_req_index recebe cpu_req_addr dos bits de 4 a 13 
		data_write = data_read;			//data_write recebe data_read
		case(cpu_req_addr[3:2])
			2'b00:data_write[31:0] = cpu_req_data;			// data_write recebe cpu_req_data dos bits de 0 a 31 
			2'b01:data_write[63:32] = cpu_req_data;			// data_writr recebe cpu_req_data dos bits de 32 a 63
			2'b10:data_write[95:64] = cpu_req_data;			// data_write recebe cpu_req_data dos bits de 64 a 95
			2'b11:data_write[127:96] = cpu_req_data;		// data_write recebe cpu_req_data dos bits de 96 a 127
		endcase
		case(cpu_req_addr[3:2])
			2'b00:v_cpu_res_data = data_read[31:0];			// v_cpu_res_data recebe data_read dos bits de 0 a 31 
			2'b01:v_cpu_res_data = data_read[63:32];		// v_cpu_res_data recebe data_read dos bits de 32 a 63
			2'b10:v_cpu_res_data = data_read[95:64];		// v_cpu_res_data recebe data_read dos bits de 64 a 95
			2'b11:v_cpu_res_data = data_read[127:96];		// v_cpu_res_data recebe data_read dos bits de 96 a 127
		endcase
		v_mem_req_addr = cpu_req_addr;					//v_mem_req_addr recebe cpu_req_addr
		v_mem_req_data = data_read;					//v_mem_req_data recebe data_read
		v_mem_req_rw = 0;						//v_mem_req_rw recebe 0
		case(rstate)
			cache_state_idle: begin
				if (cpu_req_valid)
					vstate = cache_state_compare_tag;	//vstate recebe cache_state_compare_tag
			end
			cache_state_compare_tag: begin
				if (cpu_req_addr[TAGMSB:TAGLSB] == tag_read_tag && tag_read_valid) begin
					v_cpu_res_ready = 1; 					//v_cpu_res_ready recebe 1
					if (cpu_req_rw) begin
						tag_req_we = 1; 				//tag_req_we recebe 1
						data_req_we = 1;				//data_req_we recebe 1
						tag_write_tag = tag_read_tag; 			//tag_write_tag recebe tag_read_tag
						tag_write_valid = 1;				//tag_write_valid recebe 1
						tag_write_dirty = 1;				//tag_write_dirty recebe 1
					end
					vstate = cache_state_idle;
				end
				else begin
					tag_req_we = 1;						//tag_req_we recebe 1
					tag_write_valid = 1;					//tag_write_valid recebe 1
					tag_write_tag = cpu_req_addr[TAGMSB:TAGLSB]; 		//tag_write_tag recebe cpu_req_addr com bits de TAGLSB a TAGMSB
					tag_write_dirty = cpu_req_rw; 				//tag_write_dirty recebe cpu_req_rw
					v_mem_req_valid = 1; 					//v_mem_req_valid recebe 1
					if (tag_read_valid == 1'b0 || tag_read_dirty == 1'b0)	//se tag_read_valid for igual a 1 ou tag_read_valid for igual a zero
						vstate = cache_state_allocate; 			//vstate recebe cache_state_allocate
						else begin
							v_mem_req_addr = {tag_read_tag, cpu_req_addr[TAGLSB-1:0]}; //v_mem_req_addr recebe tag_read_tag e cpu_req_addr com bits de TAGLSB-1 a 0
							v_mem_req_rw = 1; 			//v_mem_req_rw recebe 1
							vstate = cache_state_write_back; 	//vstate recebe cache_state_write_back
						end
				end
			end
			cache_state_allocate: begin
				if (mem_data_ready) begin
					vstate = cache_state_compare_tag;			//vstate recebe cache_state_compare_tag
					data_write = mem_data_data;				//data_write recebe mem_data_data
					data_req_we = 1;					//data_req_we recebe 1
				end
			end
			cache_state_write_back: begin
				if (mem_data_ready) begin
					v_mem_req_valid = 1; 					//v_mem_req_valid recebe 1
					v_mem_req_rw = 0;					//v_mem_req_rw recebe 0
					vstate = cache_state_allocate; 				//vstate recebe cache_state_allocate
				end
			end
		endcase
	end
	always @(posedge clk) begin								//Troca de estado a cada ciclo de clock
		if (rst)									//Se reset entao retorna ao estado de idle
			rstate <= cache_state_idle;
		else
			rstate <= vstate;
	end
	dm_cache_tag  ctag(									//banco de tags
		.clk(clk),
		.tag_req_we(tag_req_we),
		.tag_req_index(tag_req_index),
		.tag_write_valid(tag_write_valid),
		.tag_write_dirty(tag_write_dirty),
		.tag_write_tag(tag_write_tag),
		.tag_read_valid(tag_read_valid),
		.tag_read_dirty(tag_read_dirty),
		.tag_read_tag(tag_read_tag)
	);
	dm_cache_data cdata(									//banco de dados
		.clk(clk),
		.data_req_we(data_req_we),
		.data_req_index(data_req_index),
		.data_write(data_write),
		.data_read(data_read)
	);
endmodule
