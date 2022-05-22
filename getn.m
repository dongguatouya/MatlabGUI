function varargout=getn(H,varargin)%一般varargin和varargout是隐藏的，但可像这里作为传入参数用，

%注意为元胞类型

% 得到一个对象的多个属性

 

if max(size(H))~=1 || ~ishandle(H)

   error('Scalar Object Handle Required.')

end

varargout=get(H,varargin);

