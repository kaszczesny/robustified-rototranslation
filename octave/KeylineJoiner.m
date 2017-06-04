function [KLidx] = KeylineJoiner (mask, KLpos, KLgrad, KLidx)
% @param mask - mask of pixels that were identified as edges

edges3 = [  zeros( 1, size(mask, 2) + 2);
            zeros( size(mask, 1), 1) mask zeros( size(mask, 1), 1);
            zeros( 1, size(mask, 2) + 2);  ];          
KLpos += 1;


for kl1 = 1:size(KLpos,1)
  x = KLpos(kl1, 2);
  y = KLpos(kl1, 1);
  
  tx = -KLgrad(kl1, 1);
  ty = KLgrad(kl1, 2);
  
  
  if (ty>0)
    if (tx>0) %bottom right
      if edges3( y, x+1 ) > 0
        kl2 = find( KLpos( :, 2 ) == x+1 & KLpos( :, 1 ) == y );
      elseif edges3( y+1, x ) > 0
        kl2 = find( KLpos( :, 2 ) == x & KLpos( :, 1 ) == y+1 );
      elseif  edges3( y+1, x+1 ) > 0
        kl2 = find( KLpos( :, 2 ) == x+1 & KLpos( :, 1 ) == y+1 );
      else kl2 = 0;
      end
    else %bottom left
      if edges3( y, x-1 ) > 0
        kl2 = find( KLpos( :, 2 ) == x-1 & KLpos( :, 1 ) == y );
      elseif edges3( y+1, x ) > 0
        kl2 = find( KLpos( :, 2 ) == x & KLpos( :, 1 ) == y+1 );
      elseif edges3( y+1, x-1 ) > 0
        kl2 = find( KLpos( :, 2 ) == x-1 & KLpos( :, 1 ) == y+1 );
      else kl2 = 0;
      end
    end
    
  else
    if (tx<0) %top left
      if edges3( y, x-1 ) > 0
        kl2 = find( KLpos( :, 2 ) == x-1 & KLpos( :, 1 ) == y );
      elseif edges3( y-1, x ) > 0
        kl2 = find( KLpos( :, 2 ) == x & KLpos( :, 1 ) == y-1 );
      elseif edges3( y-1, x-1 ) > 0
        kl2 = find( KLpos( :, 2 ) == x-1 & KLpos( :, 1 ) == y-1 );
      else kl2 = 0;
      end
    else %top right
      if edges3( y, x+1 ) > 0
        kl2 = find( KLpos( :, 2 ) == x+1 & KLpos( :, 1 ) == y );
      elseif edges3( y-1, x ) > 0
        kl2 = find( KLpos( :, 2 ) == x & KLpos( :, 1 ) == y-1 );
      elseif edges3( y-1, x+1 ) > 0
        kl2 = find( KLpos( :, 2 ) == x+1 & KLpos( :, 1 ) == y-1 );
      else kl2 = 0;
      end
    end
  end
  
  KLidx(kl1,2) = kl2;
  if kl2 ~= 0
    KLidx(kl2,1) = kl1;
  end
  
end

size(KLpos)
KLpos -= 1;

% todo: get rid of 3-pixel edges now, instead of global tracker
