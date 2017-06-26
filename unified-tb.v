`timescale 1ns/100ps
module fsmtb;

	reg clk; 			
	reg rst;			
	reg cpu_req_valid;		
	reg cpu_req_rw;               
	reg [31:0] cpu_req_addr;	
	reg [31:0] cpu_req_data;      
	reg mem_data_ready;           
	reg [127:0] mem_data_data;    
	wire mem_req_valid;	     
	wire mem_req_rw;           
	wire [31:0] mem_req_addr;  
	wire [127:0] mem_req_data; 
	wire cpu_res_ready;	     
	wire [31:0] cpu_res_data;  
	dm_cache_fsm fsm(
		.clk(clk), 			
		.rst(rst),			
		.cpu_req_valid(cpu_req_valid),		
		.cpu_req_rw(cpu_req_rw),               
		.cpu_req_addr(cpu_req_addr),	
		.cpu_req_data(cpu_req_data),      
		.mem_data_ready(mem_data_ready),           
		.mem_data_data(mem_data_data),    
		.mem_req_valid(mem_req_valid),		
		.mem_req_rw(mem_req_rw),              
		.mem_req_addr(mem_req_addr),     
		.mem_req_data(mem_req_data),    
		.cpu_res_ready(cpu_res_ready),		
		.cpu_res_data(cpu_res_data)      
	);
	/*
	always @* begin
		#1 clk = ~clk;
	end
	*/
	initial begin
		$dumpfile("testfsm.vcd");
		$dumpvars(0,fsmtb);
		/*
		72	01001000
		15	00001111
		99	01100011
		1	00000001
		18	00010010
				0000000000000000000000000000000000000000000000000000010010000000
				0000000000000000000000000000000000000000000000000000000011110000
				0000000000000000000000000000000000000000000000000000011000110000
				0000000000000000000000000000000000000000000000000000000000010000
				0000000000000000000000000000000000000000000000000000000100100000
				0000000000000000000000000000000000000000000000000000000000000000
		
		*/
		#4 clk <= 0;
		rst <= 0;	//flag de resete inativa (logo nao reseta)
		cpu_req_valid <= 0; //Instrucao invalida
		cpu_req_rw <= 0;    //read
		cpu_req_addr <= 32'b00000000000000000000000000000000;
		cpu_req_data <= 0;
		mem_data_ready <= 1;
		mem_data_data <= 0;
		#4 clk <= 1;
		rst <= 0;
		cpu_req_valid <= 1; //Instrucao valida
		cpu_req_rw <= 0;    //modo leitura
		cpu_req_addr <= 32'b00000000000000000000000100100000;  //Ler no endereco 18 (sem byte offset e no primeiro bloco)
		cpu_req_data <= 0;  //se estivesse no modo escrita esse seria o dado a escrever na memoria (mas esta no modo leitura)
		mem_data_ready <= 1;//memoria esta pronta para receber instrucao/nao esta fazendo nada/concluiu processo anterior
		mem_data_data <= 0; //leitura do ultimo ciclo de clock resultou em 0 (pq nao era uma instrucao valida)/em outras palavras deu don't care
		#4 clk <= 0;
		rst <= 0;
		cpu_req_valid <= 1; //instrucao valida
		cpu_req_rw <= 0;    //modo leitura
		cpu_req_addr <= 32'b00000000000000000000000000010000;  //Ler no endereco 1 (sem byte offset e no primeiro bloco)
		cpu_req_data <= 0;  //Nao esta no modo escrita (nao tem proposito passar dado a ser escrito)
		mem_data_ready <= 1;//memoria concluiu execucao
		mem_data_data <= 59;//dado lido da instrucao anterior (do endereco 18) retornou 59
		#4 clk <= 1;
		rst <= 0;
		cpu_req_valid <= 1; 
		cpu_req_rw <= 0;    
		cpu_req_addr <= 32'b00000000000000000000011000110000;
		cpu_req_data <= ;  
		mem_data_ready <= ;
		mem_data_data <= ;
		#4 clk <= 0;
		rst <= 0;
		cpu_req_valid <= 1; 
		cpu_req_rw <= 0;    
		cpu_req_addr <= 32'b00000000000000000000000000000000;
		cpu_req_data <= ;  
		mem_data_ready <= ;
		mem_data_data <= ;
		#4 clk <= 1;
		rst <= 0;
		cpu_req_valid <= 1; 
		cpu_req_rw <= 0;    
		cpu_req_addr <= 32'b00000000000000000000000000000000;
		cpu_req_data <= ;  
		mem_data_ready <= ;
		mem_data_data <= ;
		#4 clk <= 0;
		rst <= 0;
		cpu_req_valid <= ; 
		cpu_req_rw <= 0;    
		cpu_req_addr <= 32'b00000000000000000000000000000000;
		cpu_req_data <= ;  
		mem_data_ready <= ;
		mem_data_data <= ;
		#4 clk <= 1;
		rst <= 0;
		cpu_req_valid <= 1; 
		cpu_req_rw <= 0;    
		cpu_req_addr <= 32'b00000000000000000000000000000000;
		cpu_req_data <= ;  
		mem_data_ready <= ;
		mem_data_data <= ;
		#4 clk <= 0;
		rst <= 0;
		cpu_req_valid <= 1; 
		cpu_req_rw <= 0;    
		cpu_req_addr <= ;  
		cpu_req_data <= ;  
		mem_data_ready <= ;
		mem_data_data <= ;
		#4 clk <= 1;
		rst <= 0;
		cpu_req_valid <= 1; 
		cpu_req_rw <= 0;    
		cpu_req_addr <= ;  
		cpu_req_data <= ;  
		mem_data_ready <= ;
		mem_data_data <= ;
		#100 $finish;
	end



endmodule
